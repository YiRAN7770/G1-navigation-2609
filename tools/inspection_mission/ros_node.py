#!/usr/bin/env python3
"""FAR inspection executor. Default start is disabled; no camera/LLM dependency.

Run as `python3 -m tools.inspection_mission.ros_node` from the repository root.
Requires navigation launched with inspection_mode:=true. Never clears manual
stop and never talks to SDK2. All state mutation is in a single ROS executor.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import fcntl
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .core import Pose, State, finite
from .demo import DEFAULT_CONFIG
from .io import durable_json, dwell_result, load_mission


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def pose_of(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(finite(v) for v in values):
        raise ValueError("Nonfinite odometry")
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if abs(norm - 1.0) > .05:
        raise ValueError("Invalid odometry quaternion")
    return Pose(p.x, p.y, p.z, math.atan2(2*(q.w*q.z+q.x*q.y)/norm,
                                         1-2*(q.y*q.y+q.z*q.z)/norm))


class InspectionNode(Node):
    def __init__(self):
        super().__init__("g1_inspection_executor")
        self.allowed = self.declare_parameter("allow_goal_publish", False).value
        # GICP diagnostics are produced at 0.5 Hz; ready/odometry stay at 0.5 s.
        self.diagnostic_timeout = self.declare_parameter("diagnostic_timeout", 5.0).value
        config = self.declare_parameter("config", DEFAULT_CONFIG).value
        self.output = Path(self.declare_parameter("output_dir", "run_logs/inspection").value).resolve()
        self.core, self.route = load_mission(config)
        self.run_dir = self.output / self.core.mission_id
        self.lock = open(f"/tmp/g1_inspection_domain_{os.getenv('ROS_DOMAIN_ID', '0')}.lock", "w")
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.latest, self.receipt = {}, {}
        self.raw_samples, self.map_samples = deque(maxlen=200), deque(maxlen=200)
        self.goal, self.goal_key, self.operation = None, "", None
        self.allow_drive, self.external_terminal = False, True
        self.expected_external = None
        self.external_started = False
        self.task_result = None
        self.task_start_ros_ns = 0
        self.still_since, self.best_error, self.progress_time = None, None, 0.0
        self.best_heading = None
        self.hold_started = None
        self.settle_phase, self.stationary_resets = "inactive", 0
        self.stationary_start_ros_ns = 0
        self.accepted_goal, self.sent_time, self.last_clock = False, 0.0, None
        self.health_reason = "not_checked"
        self.pool, self.job = ThreadPoolExecutor(max_workers=1), None
        self.param_futures, self.params_ok = None, False
        self.source_check_time, self.sources_ok = 0.0, False
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.velocity_pub = self.create_publisher(TwistStamped, "/cmd_vel_fastlio_far_inspection", 1)
        self.status_pub = self.create_publisher(String, "/inspection/status", 10)
        self.request_pub = self.create_publisher(String, "/inspection/task/request", 10)
        self.gate_client = self.create_client(GetParameters, "/fastlio_far_motion_gate/get_parameters")
        self.map_client = self.create_client(GetParameters, "/prior_map_localizer/get_parameters")
        self.create_subscription(Odometry, "/state_estimation", lambda m: self.odom("odom", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/Odometry", lambda m: self.odom("raw_odom", m), qos_profile_sensor_data)
        self.create_subscription(Bool, "/localization_ready", lambda m: self.remember("ready", m.data), 10)
        self.create_subscription(DiagnosticArray, "/localization_diagnostics", lambda m: self.remember("diagnostics", m), 10)
        self.create_subscription(String, "/fastlio_far/motion_status", lambda m: self.remember("gate", m.data), 10)
        for key, topic in [("nav", "/fastlio_far/goal_status"), ("far", "/far_planner/status"),
                           ("local", "/local_planner/status")]:
            self.create_subscription(String, topic, lambda m, k=key: self.json_status(k, m), 10)
        self.create_subscription(TwistStamped, "/cmd_vel_fastlio_far_raw", self.command, 1)
        self.create_subscription(PoseStamped, "/goal_pose", self.observe_goal, 10)
        self.create_subscription(String, "/inspection/task/result", self.external_result, 10)
        for name in ("start", "pause", "resume", "cancel"):
            self.create_service(Trigger, "/inspection/" + name,
                                lambda req, res, op=name: self.service(op, res))
        self.timer = self.create_timer(.05, self.update)
        self.get_logger().info("Ready; start is opt-in. Task mode is dwell; no recognition is performed.")

    def remember(self, key, value):
        self.latest[key], self.receipt[key] = value, time.monotonic()

    def fresh(self, key, timeout=.5):
        return time.monotonic() - self.receipt.get(key, -math.inf) <= timeout

    def stamp_fresh(self, stamp, timeout=.5):
        ns = stamp_ns(stamp)
        return ns > 0 and -.1 <= (self.get_clock().now().nanoseconds - ns)/1e9 <= timeout

    def odom(self, key, msg):
        if not self.stamp_fresh(msg.header.stamp) or (key == "odom" and msg.header.frame_id != "map"):
            return
        samples = self.map_samples if key == "odom" else self.raw_samples
        ns = stamp_ns(msg.header.stamp)
        if samples and ns <= samples[-1][0]:
            return
        try:
            pose = pose_of(msg)
        except ValueError:
            return
        samples.append((ns, pose))
        self.remember(key, msg)
        # A short speed excursion between two timer ticks must also break the
        # continuous interval. ROS callbacks execute serially with update().
        if (self.still_since is not None and self.operation is not None
                and self.operation.kind in ("hold_and_verify", "inspect")
                and not self.stationary()):
            self.reset_stationary(time.monotonic())

    def json_status(self, key, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.remember(key, data)
        except ValueError:
            pass

    def service(self, op, response):
        try:
            now = time.monotonic()
            if op == "start":
                if not self.allowed:
                    raise RuntimeError("Start disabled: set allow_goal_publish:=true after staging checks")
                self.core.start(now)
            elif op == "resume":
                self.core.resume(now)
            elif op == "cancel":
                self.allow_drive = False
                self.core._fail("operator_canceled", now)
            else:
                self.allow_drive = False
                self.core.pause(now)
            response.success, response.message = True, self.core.state.value
        except (ValueError, RuntimeError) as error:
            response.success, response.message = False, str(error)
        return response

    def zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vehicle"
        self.velocity_pub.publish(msg)

    def command(self, msg):
        if (self.allow_drive and self.core.state in (State.NAVIGATING, State.RETURNING)
                and self.stamp_fresh(msg.header.stamp, .25)):
            # Preserve the original timestamp. The original gate applies health,
            # manual stop, freshness, finite-value validation and speed limits.
            self.velocity_pub.publish(msg)
        else:
            self.zero()

    def observe_goal(self, msg):
        if self.core.state not in (State.IDLE, State.PRECHECK, *self.core.TERMINAL):
            if str(stamp_ns(msg.header.stamp)) != self.goal_key:
                self.allow_drive = False
                self.core.pause(time.monotonic(), "another_goal_source")

    def external_result(self, msg):
        try:
            result = json.loads(msg.data)
            if not isinstance(result, dict) or not self.expected_external:
                return
            if any(result.get(k) != v for k, v in self.expected_external.items()):
                return
            if result.get("status") in ("succeeded", "failed", "canceled"):
                self.external_terminal = True
                self.task_result = result
        except ValueError:
            pass

    def healthy(self):
        now = time.monotonic()
        if now - self.source_check_time > .5:
            topics = ["/state_estimation", "/Odometry", "/localization_ready", "/fastlio_far/goal_status",
                      "/cmd_vel_fastlio_far_raw", "/cmd_vel_fastlio_far_inspection",
                      "/cmd_vel_fastlio_far_safe"]
            self.sources_ok = all(self.count_publishers(t) == 1 for t in topics)
            self.source_check_time = now
        if not self.sources_ok:
            return "missing_or_duplicate_sources"
        if not self.params_ok:
            return "gate_or_map_not_verified"
        for key in ("odom", "raw_odom", "ready", "gate"):
            if not self.fresh(key):
                return key + "_stale"
        if not self.fresh("diagnostics", self.diagnostic_timeout):
            return "diagnostics_stale"
        if not self.stamp_fresh(self.latest["diagnostics"].header.stamp, self.diagnostic_timeout):
            return "diagnostics_stamp_stale"
        if not all(self.stamp_fresh(self.latest[k].header.stamp) for k in ("odom", "raw_odom")):
            return "sensor_stamp_stale"
        if self.latest.get("ready") is not True:
            return "localization_not_ready"
        statuses = self.latest["diagnostics"].status
        if not any(s.name.endswith("prior_map_localizer") and s.level in (0, bytes([0])) and s.message == "localized" for s in statuses):
            return "localization_diagnostics_unhealthy"
        if self.latest["gate"] not in ("ready", "disabled"):
            return "gate:" + self.latest["gate"]
        try:
            transform = self.tf.lookup_transform("map", "body", Time())
            if not self.stamp_fresh(transform.header.stamp):
                return "tf_stale"
        except TransformException:
            return "tf_unavailable"
        return "ready"

    def stationary(self):
        if not self.fresh("raw_odom") or not self.fresh("odom") or len(self.raw_samples) < 3:
            return False
        end, pose = self.raw_samples[-1]
        window = [(stamp, p) for stamp, p in self.raw_samples if end - stamp <= 650_000_000]
        if len(window) < 3 or (end-window[0][0])/1e9 < .45:
            return False
        # Evaluate displacement over the window, not only reported zero twist.
        for stamp, old in window:
            dt = (end-stamp)/1e9
            if dt >= .2 and (math.hypot(pose.x-old.x, pose.y-old.y)/dt >= self.core.policy.stopped_linear_speed
                             or abs(wrap(pose.yaw-old.yaw))/dt > .05):
                return False
        for key in ("odom", "raw_odom"):
            twist = self.latest[key].twist.twist
            values = (twist.linear.x, twist.linear.y, twist.angular.z)
            if (not all(finite(v) for v in values)
                    or math.hypot(*values[:2]) >= self.core.policy.stopped_linear_speed
                    or abs(values[2]) > .05):
                return False
        return True

    def at_target(self):
        return self.position_at_target() and self.stationary()

    def position_at_target(self):
        if self.goal is None or len(self.map_samples) < 3 or not self.fresh("odom"):
            return False
        # Three distinct, post-goal localization samples, each inside both bounds.
        for stamp, pose in list(self.map_samples)[-3:]:
            if (stamp <= int(self.goal_key) or math.hypot(pose.x-self.goal.x, pose.y-self.goal.y) > .10
                    or abs(wrap(pose.yaw-self.goal.yaw)) > math.radians(20)):
                return False
        return True

    def matching_status(self):
        nav = self.latest.get("nav", {})
        if not self.fresh("nav") or nav.get("goal_stamp_ns") != self.goal_key or self.goal is None:
            return False
        try:
            return (abs(nav["target_x"]-self.goal.x) < 1e-6
                    and abs(nav["target_y"]-self.goal.y) < 1e-6
                    and abs(wrap(nav["target_yaw"]-self.goal.yaw)) < 1e-6)
        except (KeyError, TypeError, ValueError):
            return False

    def arrived(self):
        nav, far = self.latest.get("nav", {}), self.latest.get("far", {})
        return (self.matching_status() and self.at_target() and self.fresh("far")
                and nav.get("phase") == "reached" and nav.get("reached") is True
                and nav.get("reason") == "arrival_latched"
                and far.get("goal_stamp_ns") == self.goal_key and far.get("state") == "succeeded")

    def begin_effect(self, effect, now):
        # Carry the verified stationary interval only into the immediately
        # following task. Dwell consumes that interval, not another three seconds.
        carry = (effect.kind == "inspect" and self.operation is not None
                 and self.operation.kind == "hold_and_verify"
                 and self.operation.point_id == effect.point_id
                 and self.settle_phase == "verified")
        if not carry:
            self.still_since, self.hold_started = None, None
            self.stationary_start_ros_ns = 0
            self.settle_phase, self.stationary_resets = "inactive", 0
        self.operation = effect
        self.allow_drive = False
        self.zero()
        if effect.kind == "precheck":
            self.params_ok, self.param_futures = False, None
        elif effect.kind == "navigate":
            self.goal = Pose(**effect.payload["pose"])
            msg = PoseStamped()
            msg.header.frame_id, msg.header.stamp = "map", self.get_clock().now().to_msg()
            key = str(stamp_ns(msg.header.stamp))
            if int(key) <= int(self.goal_key or "0"):
                raise ValueError("ROS clock must advance for every new goal")
            self.goal_key, self.sent_time = key, now
            self.accepted_goal, self.best_error, self.progress_time = False, None, now
            self.best_heading = None
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.goal.x, self.goal.y, self.goal.z
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(self.goal.yaw/2), math.cos(self.goal.yaw/2)
            self.goal_pub.publish(msg)  # Exactly one publish per attempt.
        elif effect.kind == "hold_and_verify":
            self.hold_started, self.settle_phase = now, "buffer"
            self.get_logger().info(f"SETTLE_BUFFER_START point={effect.point_id} seconds={self.core.policy.settle_seconds}")
        elif effect.kind == "inspect":
            self.task_result = None
            self.external_started = False
            self.task_start_ros_ns = self.get_clock().now().nanoseconds
            if effect.payload["task"]["kind"] == "external":
                self.expected_external = {"mission_id": effect.mission_id, "point_id": effect.point_id,
                                          "request_id": effect.token, "map_id": self.core.map_id}
        elif effect.kind in ("commit", "finish_record"):
            path = self.run_dir / (effect.point_id + ".json" if effect.kind == "commit" else "complete.json")
            data = effect.payload["record"] if effect.kind == "commit" else effect.payload
            reply = "committed" if effect.kind == "commit" else "finished"
            response = {"key": effect.payload.get("key"), "durable": True}
            self.job = (self.pool.submit(durable_json, path, data), effect.token, reply, response)
        elif effect.kind in ("cancel_and_hold", "emergency_hold"):
            if not self.external_terminal and self.expected_external:
                self.request_pub.publish(String(data=json.dumps({**self.expected_external, "command": "cancel"})))
        if effect.kind in ("mission_complete", "emergency_hold"):
            self.pool.submit(durable_json, self.run_dir / "terminal.json", {
                "mission_id": self.core.mission_id, "state": self.core.state.value,
                "reason": self.core.reason, "history": self.core.history,
                "completed": [r["point_id"] for r in self.core.records],
                "recognition_status": "not_configured"})
        self.get_logger().info(f"{self.core.state.value}: {effect.point_id} {effect.kind}")

    def reset_stationary(self, now):
        if self.still_since is not None:
            self.stationary_resets += 1
            self.get_logger().info(f"STATIONARY_RESET point={self.operation.point_id} elapsed={now-self.still_since:.3f} count={self.stationary_resets}")
        self.still_since = None
        self.stationary_start_ros_ns = 0
        self.settle_phase = "detecting"
        if (self.operation.kind == "inspect" and self.external_started
                and self.operation.payload["task"]["kind"] == "external"):
            self.core.pause(now, "moved_during_external_task")

    def navigation_progress(self, nav, now):
        # Translation may initially increase straight-line goal distance while
        # following a curve toward a behind-robot waypoint. Preserve the distance
        # watchdog, but also recognize measured, commanded heading improvement.
        phase = nav.get("phase")
        error = abs(nav.get("yaw_error", math.inf)) if phase == "align" else nav.get("distance", math.inf)
        if finite(error):
            if self.best_error is None or phase != self.best_error[0] or error < self.best_error[1]-.03:
                self.best_error, self.progress_time = (phase, error), now
                self.best_heading = None
        if phase != "approach":
            self.best_heading = None
            return
        local = self.latest.get("local", {})
        if (not self.fresh("local") or not self.fresh("odom") or self.goal is None
                or local.get("goal_stamp_ns") != self.goal_key
                or local.get("final_goal_valid") is not True
                or local.get("reason") != "path_available"
                or local.get("rotation_clear") is not True):
            return
        values = [local.get(k) for k in ("target_x", "target_y", "target_yaw", "waypoint_angle")]
        if (not all(finite(v) for v in values)
                or abs(values[0]-self.goal.x) > 1e-6 or abs(values[1]-self.goal.y) > 1e-6
                or abs(wrap(values[2]-self.goal.yaw)) > 1e-6):
            return
        heading = wrap(values[3] - (math.pi if nav.get("drive_mode") == "reverse" else 0.))
        yaw = pose_of(self.latest["odom"]).yaw
        if self.best_heading is None:
            self.best_heading = (heading, yaw)
            return
        old_heading, old_yaw = self.best_heading
        rotation = wrap(yaw-old_yaw)
        commanded = nav.get("raw_wz")
        if (abs(heading) < abs(old_heading)-.03 and abs(rotation) > .03
                and rotation*old_heading > 0 and finite(commanded) and rotation*commanded > 0):
            self.best_heading, self.progress_time = (heading, yaw), now
            self.get_logger().info(f"NAV_PROGRESS turn heading_error={abs(heading):.3f} rotation={rotation:.3f}")

    def advance_operation(self, now):
        effect = self.operation
        if effect is None or effect.token != self.core.token:
            return
        if effect.kind == "precheck":
            if self.param_futures is None and self.gate_client.service_is_ready() and self.map_client.service_is_ready():
                self.param_futures = (
                    self.gate_client.call_async(GetParameters.Request(names=["input_topic"])),
                    self.map_client.call_async(GetParameters.Request(names=["map_path"])))
            if self.param_futures and all(f.done() for f in self.param_futures):
                gate, localizer = [f.result().values[0].string_value for f in self.param_futures]
                if gate != "/cmd_vel_fastlio_far_inspection" or Path(localizer).resolve() != Path(self.route["prior_pcd"]).resolve():
                    self.core.handle("failed", effect.token, now, {"reason": "wrong_gate_input_or_map"})
                    return
                self.params_ok = True
            subscribers = {e.node_name for e in self.get_subscriptions_info_by_topic("/goal_pose")}
            if (self.healthy() == "ready" and self.stationary() and self.job is None
                    and {"far_planner", "localPlanner", "pathFollower"} <= subscribers):
                home = self.core.home or pose_of(self.latest["odom"])
                result = {"map_id": self.core.map_id, "home": asdict(home), "all_checks_ok": True}
                # Snapshot home and configuration durably before the first goal.
                data = {"mission_id": self.core.mission_id, **result, "route": self.route,
                        "tasks": [asdict(p) for p in self.core.points], "policy": asdict(self.core.policy)}
                self.job = (self.pool.submit(durable_json, self.run_dir / "start.json", data),
                            effect.token, "precheck_ok", result)
        elif effect.kind == "navigate":
            if self.arrived():
                self.allow_drive = False
                self.core.handle("nav_succeeded", effect.token, now)
                return
            if self.matching_status():
                self.accepted_goal = True
                self.allow_drive = True
                nav = self.latest["nav"]
                self.navigation_progress(nav, now)
            else:
                self.allow_drive = False
                if self.accepted_goal:
                    self.core.pause(now, "current_goal_status_lost_or_replaced")
                    return
            if not self.accepted_goal and now-self.sent_time > 5.0:
                self.core.handle("failed", effect.token, now, {"reason": "goal_ack_timeout"})
            elif now-self.progress_time > 15.0:
                self.core.handle("failed", effect.token, now, {"reason": "no_progress"})
        elif effect.kind in ("hold_and_verify", "inspect"):
            if not self.position_at_target():
                self.still_since = None
                self.core.pause(now, "left_target_during_settle_or_task")
                return
            if effect.kind == "hold_and_verify" and now-self.hold_started < self.core.policy.settle_seconds:
                return
            if self.settle_phase == "buffer":
                self.settle_phase = "detecting"
                self.get_logger().info(f"SETTLE_BUFFER_END point={effect.point_id}; continuous stationary detection begins")
            if not self.stationary():
                self.reset_stationary(now)
                return
            if self.still_since is None:
                self.stationary_start_ros_ns = self.get_clock().now().nanoseconds
            self.still_since = now if self.still_since is None else self.still_since
            elapsed = now-self.still_since
            if effect.kind == "hold_and_verify" and elapsed >= self.core.policy.stationary_seconds:
                self.settle_phase = "verified"
                self.get_logger().info(f"STATIONARY_CONFIRMED point={effect.point_id} stationary={elapsed:.3f} total={now-self.hold_started:.3f} resets={self.stationary_resets}")
                self.core.handle("stopped", effect.token, now, {"fresh_stationary_at_target": True})
            elif effect.kind == "inspect":
                task = effect.payload["task"]
                if task["kind"] == "dwell" and elapsed >= max(task["dwell_seconds"], self.core.policy.stationary_seconds):
                    result = dwell_result(effect, elapsed)
                    result.update(goal_stamp_ns=self.goal_key, target_pose=asdict(self.goal),
                                  actual_pose=asdict(pose_of(self.latest["odom"])),
                                  task_start_stamp_ns=self.task_start_ros_ns,
                                  task_end_stamp_ns=self.get_clock().now().nanoseconds,
                                  stationary_start_stamp_ns=self.stationary_start_ros_ns,
                                  settle_seconds=self.core.policy.settle_seconds,
                                  stationary_resets=self.stationary_resets,
                                  stopped_linear_speed=self.core.policy.stopped_linear_speed)
                    self.core.handle("inspection_succeeded", effect.token, now, result)
                elif task["kind"] == "external" and elapsed >= self.core.policy.stationary_seconds:
                    if not self.external_started:
                        self.external_terminal, self.external_started = False, True
                        self.request_pub.publish(String(data=json.dumps({
                            **self.expected_external, "command": "start", "schema_version": 1,
                            "goal_stamp_ns": self.goal_key,
                            "task_start_stamp_ns": self.get_clock().now().nanoseconds})))
                    elif self.task_result:
                        event = "inspection_succeeded" if self.task_result.get("status") == "succeeded" else "failed"
                        self.core.handle(event, effect.token, now, self.task_result)
                        self.task_result = None
        elif effect.kind == "cancel_and_hold" and self.external_terminal and self.stationary():
            self.core.handle("quiesced", effect.token, now, {"workers_terminal": True, "fresh_stationary": True})

    def update(self):
        now = time.monotonic()
        try:
            ros_now = self.get_clock().now().nanoseconds
            if self.last_clock is not None and ros_now < self.last_clock:
                self.allow_drive = False
                self.core.pause(now, "ROS_clock_rollback")
            self.last_clock = ros_now
            self.core.tick(now)
            self.health_reason = self.healthy()
            active = self.core.state not in (State.IDLE, State.PRECHECK, State.PAUSED, State.CANCELING, *self.core.TERMINAL)
            if active and self.health_reason != "ready":
                self.allow_drive = False
                self.core.pause(now, self.health_reason)
            if self.core.state in (State.COMMITTING, State.FINISHING) and not self.at_target():
                self.allow_drive = False
                self.core.pause(now, "moved_during_storage")
            if self.job and self.job[0].done():
                future, token, reply, data = self.job
                # Do not accept a stale precheck snapshot if health changed during IO.
                if reply != "precheck_ok" or self.healthy() == "ready":
                    self.job = None
                    try:
                        future.result()
                        self.core.handle(reply, token, now, data)
                    except Exception as error:
                        self.core.handle("failed", token, now, {"reason": "storage:" + str(error)})
            for effect in self.core.drain_effects():
                self.begin_effect(effect, now)
            if self.health_reason == "ready" or self.core.state in (State.PRECHECK, State.CANCELING):
                self.advance_operation(now)
            if self.core.state not in (State.NAVIGATING, State.RETURNING):
                self.allow_drive = False
            if not self.allow_drive:
                self.zero()
            status = {"mission_id": self.core.mission_id, "state": self.core.state.value,
                      "point_id": self.core.point_id, "request_id": self.core.token,
                      "goal_stamp_ns": self.goal_key, "completed": [r["point_id"] for r in self.core.records],
                      "reason": self.core.reason, "health": self.health_reason,
                      "settle_phase": self.settle_phase,
                      "settle_elapsed": 0.0 if self.hold_started is None else now-self.hold_started,
                      "stationary_elapsed": 0.0 if self.still_since is None else now-self.still_since,
                      "stationary_resets": self.stationary_resets,
                      "allow_drive": self.allow_drive, "recognition_status": "not_configured"}
            self.status_pub.publish(String(data=json.dumps(status)))
        except Exception as error:
            self.allow_drive = False
            self.zero()
            self.core._fail("executor_exception:" + str(error), now)
            self.get_logger().error(str(error))

    def close(self):
        self.allow_drive = False
        self.zero()
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.lock.close()


def main():
    rclpy.init()
    node = None
    try:
        node = InspectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if rclpy.ok():
                node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
