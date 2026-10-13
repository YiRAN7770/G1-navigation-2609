"""Localhost ROS integration with synthetic navigation/odometry, actual safety gate.

Never launch SDK2. A nonzero Domain and ROS_LOCALHOST_ONLY=1 are mandatory.
This tests coordination and watchdog behavior, NOT real navigation or perception.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import yaml

import rclpy
from rclpy.executors import SingleThreadedExecutor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .demo import DEFAULT_CONFIG


ROOT = Path(__file__).resolve().parents[2]
GATE = Path(os.environ.get(
    "G1_MOTION_GATE_BIN",
    ROOT / "ros2_ws/install/vehicle_simulator/lib/vehicle_simulator/fastlio_far_motion_gate",
))


class RosMissionTest(unittest.TestCase):
    def setUp(self):
        if os.getenv("ROS_LOCALHOST_ONLY") != "1" or int(os.getenv("ROS_DOMAIN_ID", "0")) == 0:
            raise RuntimeError("Refusing synthetic sensors outside an isolated localhost ROS domain")
        self.tmp = tempfile.TemporaryDirectory(prefix="g1-inspection-ros-")
        self.folder = Path(self.tmp.name)
        config = yaml.safe_load(Path(DEFAULT_CONFIG).read_text())
        route_path = Path(config["route"])
        if not route_path.is_absolute():
            route_path = Path(DEFAULT_CONFIG).resolve().parent / route_path
        config["route"] = str(route_path.resolve())
        if not self._testMethodName.startswith("test_real_duration"):
            config["policy"].update(settle_seconds=.2, stationary_seconds=.3)
        if not self._testMethodName.startswith("test_real_duration"):
            for spec in config["tasks"].values():
                spec["dwell_seconds"] = .3
        if self._testMethodName == "test_p2_stall_stops_without_skip":
            config["policy"]["nav_retries"] = 0
        (self.folder / "mission.yaml").write_text(yaml.safe_dump(config))
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = rclpy.create_node("synthetic_inspection_fixture")
        self.nodes = [self.node]
        for name in ("far_planner", "localPlanner", "pathFollower", "prior_map_localizer"):
            n = rclpy.create_node(name)
            self.nodes.append(n)
            if name != "prior_map_localizer":
                n.create_subscription(PoseStamped, "/goal_pose", self.goal_received if name == "pathFollower" else lambda m: None, 10)
            else:
                route_file = Path(config["route"])
                route = yaml.safe_load(route_file.read_text())
                map_path = Path(route["prior_pcd"])
                if not map_path.is_absolute():
                    map_path = route_file.parent / map_path
                n.declare_parameter("map_path", str(map_path.resolve()))
        for n in self.nodes:
            self.executor.add_node(n)
        self.tf = TransformBroadcaster(self.node)
        self.pubs = {key: self.node.create_publisher(typ, topic, 10) for key, typ, topic in [
            ("odom", Odometry, "/state_estimation"), ("raw_odom", Odometry, "/Odometry"),
            ("cloud", PointCloud2, "/registered_scan"), ("path", NavPath, "/fastlio_far/local_path"),
            ("raw", TwistStamped, "/cmd_vel_fastlio_far_raw"), ("ready", Bool, "/localization_ready"),
            ("diag", DiagnosticArray, "/localization_diagnostics"),
            ("nav", String, "/fastlio_far/goal_status"), ("far", String, "/far_planner/status")]}
        self.status, self.safe, self.goals, self.states = {}, [], [], []
        self.state_times = []
        self.node.create_subscription(String, "/inspection/status", self.status_received, 10)
        self.node.create_subscription(TwistStamped, "/cmd_vel_fastlio_far_safe", lambda m: self.safe.append((time.monotonic(), m.twist.linear.x)), 20)
        self.home = (0.37, -.61, .1, .24)
        self.pose = self.home
        self.goal, self.goal_time = None, 0.
        self.freeze_navigation, self.localize = False, True
        self.linear_speed = 0.
        self.procs, self.logs = [], []
        self.launch([str(GATE), "--ros-args", "-p", "enable_motion:=true", "-p", "require_localization_ready:=true",
                     "-p", "input_topic:=/cmd_vel_fastlio_far_inspection", "-p", "path_topic:=/fastlio_far/local_path"], "gate")
        self.mission_proc = self.launch(["python3", "-m", "tools.inspection_mission.ros_node", "--ros-args",
                                        "-p", "allow_goal_publish:=true", "-p", "config:="+str(self.folder/"mission.yaml"),
                                        "-p", "output_dir:="+str(self.folder/"results")], "mission")
        self.client = self.node.create_client(Trigger, "/inspection/start")
        self.addCleanup(self.cleanup)

    def launch(self, command, name):
        log = (self.folder/(name+".log")).open("w")
        self.logs.append(log)
        proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        self.procs.append(proc)
        return proc

    def cleanup(self):
        for proc in reversed(self.procs):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        for log in self.logs:
            log.close()
        for n in self.nodes:
            self.executor.remove_node(n)
            n.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        # Keep logs in the workspace evidence directory when requested.
        output = os.getenv("G1_INSPECTION_TEST_LOG_DIR")
        if output:
            import shutil
            destination = Path(output)/self._testMethodName
            shutil.copytree(self.folder, destination, dirs_exist_ok=True)
        self.tmp.cleanup()

    def goal_received(self, msg):
        self.goal, self.goal_time = msg, time.monotonic()
        self.goals.append(msg)

    def status_received(self, msg):
        self.status = json.loads(msg.data)
        self.states.append((self.status["state"], self.status["point_id"]))
        self.state_times.append((time.monotonic(), self.status.copy()))

    def feed(self):
        stamp = self.node.get_clock().now().to_msg()
        elapsed = time.monotonic()-self.goal_time
        reached = bool(self.goal and elapsed > .9 and not self.freeze_navigation)
        if self.goal and elapsed > .15 and not self.freeze_navigation:
            p, q = self.goal.pose.position, self.goal.pose.orientation
            self.pose = (p.x, p.y, p.z, 2*math.atan2(q.z, q.w))
        for key, frame in (("odom", "map"), ("raw_odom", "camera_init")):
            msg = Odometry()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, frame, "body"
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = self.pose[:3]
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(self.pose[3]/2), math.cos(self.pose[3]/2)
            msg.twist.twist.linear.x = self.linear_speed
            self.pubs[key].publish(msg)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "map", "body"
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        cloud = PointCloud2()
        cloud.header.stamp, cloud.width, cloud.height, cloud.data = stamp, 1, 1, [0]
        self.pubs["cloud"].publish(cloud)
        self.pubs["ready"].publish(Bool(data=self.localize))
        diag = DiagnosticArray()
        diag.header.stamp = stamp
        diag.status = [DiagnosticStatus(level=bytes([0]), name="g1_fast_lio_bringup/prior_map_localizer", message="localized")]
        self.pubs["diag"].publish(diag)
        cmd = TwistStamped()
        cmd.header.stamp = stamp
        cmd.twist.linear.x = .15 if self.goal and not reached else 0.
        self.pubs["raw"].publish(cmd)
        key = str(self.goal.header.stamp.sec*10**9+self.goal.header.stamp.nanosec) if self.goal else "0"
        nav = {"goal_stamp_ns": key, "target_x": self.goal.pose.position.x if self.goal else 0.,
               "target_y": self.goal.pose.position.y if self.goal else 0.,
               "target_yaw": 2*math.atan2(self.goal.pose.orientation.z, self.goal.pose.orientation.w) if self.goal else 0.,
               "phase": "reached" if reached else "approach", "reached": reached,
               "reason": "arrival_latched" if reached else "tracking", "distance": 0. if reached else 1., "yaw_error": 0.}
        self.pubs["nav"].publish(String(data=json.dumps(nav)))
        self.pubs["far"].publish(String(data=json.dumps({"goal_stamp_ns": key, "state": "succeeded" if reached else "planning"})))

    def spin_until(self, condition, seconds=10):
        deadline = time.monotonic()+seconds
        next_feed = 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_feed:
                self.feed()
                next_feed = time.monotonic()+.02
            self.executor.spin_once(timeout_sec=.005)
            if condition():
                return
        logs = "\n".join(p.read_text()[-2000:] for p in self.folder.glob("*.log"))
        self.fail(f"Timeout; status={self.status}; goals={len(self.goals)}\n{logs}")

    def start(self):
        self.spin_until(lambda: self.client.service_is_ready() and self.status.get("state") == "IDLE")
        future = self.client.call_async(Trigger.Request())
        self.spin_until(future.done)
        self.assertTrue(future.result().success, future.result().message)

    def test_full_sequence_and_home_with_no_camera(self):
        self.start()
        self.spin_until(lambda: self.status.get("state") == "COMPLETED", 30)
        self.assertEqual(len(self.goals), 5)
        self.assertEqual(self.status["completed"], ["P1", "P2", "P3", "P4"])
        self.assertAlmostEqual(self.goals[-1].pose.position.x, self.home[0])
        self.assertAlmostEqual(self.goals[-1].pose.position.y, self.home[1])
        self.assertAlmostEqual(2*math.atan2(self.goals[-1].pose.orientation.z, self.goals[-1].pose.orientation.w), self.home[3])
        for point in ("P1", "P2", "P3", "P4"):
            self.assertIn(("VERIFY_STOP", point), self.states)
        self.assertEqual(self.status["recognition_status"], "not_configured")
        self.assertTrue(list((self.folder/"results").glob("*/complete.json")))

    def test_real_duration_full_sequence_and_home(self):
        self.start()
        self.spin_until(lambda: self.status.get("state") == "COMPLETED", 65)
        self.assertEqual(len(self.goals), 5)
        self.assertEqual(self.status["completed"], ["P1", "P2", "P3", "P4"])
        run = next((self.folder/"results").iterdir())
        for point in ("P1", "P2", "P3", "P4", "HOME"):
            buffer_start = next(t for t,s in self.state_times
                                if s["point_id"] == point and s.get("settle_phase") == "buffer")
            state = "FINISHING" if point == "HOME" else "INSPECTING"
            next_stage = next(t for t,s in self.state_times if s["point_id"] == point and s["state"] == state)
            # Status observations have a 50 ms sampling offset; the online timer
            # and durable point timestamps provide the exact duration evidence.
            self.assertGreaterEqual(next_stage-buffer_start, 7.95)
            if point != "HOME":
                record = json.loads((run/(point+".json")).read_text())
                self.assertEqual(record["settle_seconds"], 5.)
                self.assertGreaterEqual(record["stationary_seconds"], 3.)
                self.assertEqual(record["stopped_linear_speed"], .05)
                self.assertEqual(record["recognition_status"], "not_configured")
                self.assertGreaterEqual((record["task_start_stamp_ns"]-record["stationary_start_stamp_ns"])/1e9, 3.)
        home = self.goals[-1].pose
        self.assertAlmostEqual(home.position.x, self.home[0])
        self.assertAlmostEqual(home.position.y, self.home[1])
        self.assertTrue((run/"complete.json").exists())
        cutoff = time.monotonic()
        self.spin_until(lambda: time.monotonic()-cutoff > .5)
        self.assertTrue(all(v == 0. for t,v in self.safe if t > cutoff))
        self.assertFalse(self.status["allow_drive"])

    def test_p2_stall_stops_without_skip(self):
        self.start()
        self.spin_until(lambda: len(self.goals) == 2, 15)
        self.freeze_navigation = True
        self.spin_until(lambda: self.status.get("state") == "FAILED", 22)
        self.assertIn("no_progress", self.status["reason"])
        self.assertEqual(self.status["completed"], ["P1"])
        self.assertEqual(self.status["point_id"], "P2")
        self.assertEqual(len(self.goals), 2)
        self.assertFalse(self.status["allow_drive"])
        cutoff = time.monotonic()
        self.spin_until(lambda: time.monotonic()-cutoff > .5)
        self.assertTrue(all(v == 0. for t,v in self.safe if t > cutoff))

    def test_executor_death_closes_gate_while_raw_commands_continue(self):
        self.freeze_navigation = True
        self.start()
        self.spin_until(lambda: any(v > .1 for _, v in self.safe), 10)
        self.mission_proc.kill()
        self.mission_proc.wait(timeout=5)
        cutoff = time.monotonic()+.8
        self.spin_until(lambda: len([v for t, v in self.safe if t > cutoff]) > 5, 5)
        self.assertTrue(all(v == 0. for t, v in self.safe if t > cutoff))
        self.assertEqual(len(self.goals), 1)

    def test_localization_loss_pauses_without_next_goal(self):
        self.freeze_navigation = True
        self.start()
        self.spin_until(lambda: self.status.get("allow_drive") is True, 10)
        self.localize = False
        self.spin_until(lambda: self.status.get("state") == "PAUSED", 8)
        self.assertEqual(len(self.goals), 1)
        self.assertFalse(self.status["allow_drive"])

    def test_real_duration_and_speed_reset_before_next_goal(self):
        self.start()
        self.spin_until(lambda: self.status.get("settle_phase") == "buffer")
        buffer_seen = time.monotonic()
        self.spin_until(lambda: self.status.get("stationary_elapsed", 0.) > 2., 10)
        self.assertGreaterEqual(self.status["settle_elapsed"], 7.)
        self.assertEqual(len(self.goals), 1)
        self.linear_speed = .06
        self.spin_until(lambda: self.status.get("stationary_resets", 0) >= 1)
        self.assertEqual(self.status["state"], "VERIFY_STOP")
        self.assertEqual(self.status["stationary_elapsed"], 0.)
        reset_seen = time.monotonic()
        self.linear_speed = 0.
        self.spin_until(lambda: len(self.goals) >= 2, 8)
        self.assertGreaterEqual(time.monotonic()-reset_seen, 3.)
        self.assertGreaterEqual(time.monotonic()-buffer_seen, 10.)
        record = json.loads(next((self.folder/"results").glob("*/P1.json")).read_text())
        self.assertGreaterEqual(record["stationary_seconds"], 3.)
        self.assertGreaterEqual(record["stationary_resets"], 1)

    def test_localization_loss_during_settling_pauses(self):
        self.start()
        self.spin_until(lambda: self.status.get("state") == "VERIFY_STOP")
        self.localize = False
        self.spin_until(lambda: self.status.get("state") == "PAUSED", 8)
        self.assertEqual(len(self.goals), 1)
        self.assertFalse(self.status["allow_drive"])


if __name__ == "__main__":
    unittest.main()
