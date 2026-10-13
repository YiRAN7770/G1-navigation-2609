"""Production adapter timing with a deterministic clock; no ROS graph/actuator."""
from collections import deque
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from nav_msgs.msg import Odometry
from .core import Effect, Policy, Pose, State
from .ros_node import InspectionNode


class SettlingTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.events, self.pauses, self.logs = [], [], []
        self.node = n = InspectionNode.__new__(InspectionNode)
        n.core = SimpleNamespace(policy=Policy(), token="hold", state=State.VERIFY_STOP,
                                 map_id="test", handle=lambda *a: self.events.append(a),
                                 pause=lambda *a: self.pauses.append(a))
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.now*1e9)))
        n.get_logger = lambda: SimpleNamespace(info=self.logs.append)
        n.zero = Mock()
        n.operation, n.still_since = None, None
        n.settle_phase, n.stationary_resets = "inactive", 0
        n.external_terminal, n.external_started = True, False
        n.expected_external, n.task_result = None, None
        n.request_pub = SimpleNamespace(publish=Mock())
        n.position_at_target = lambda: True
        n.stationary = lambda: True
        n.goal, n.goal_key = Pose(0., 0., 0., 0.), "1"
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.
        n.latest = {"odom": odom}
        n.begin_effect(self.effect("hold_and_verify", "hold"), self.now)

    def effect(self, kind, token, point="P1", task="dwell"):
        return Effect(kind, token, "mission", point,
                      {"map_id": "test", "task": {"kind": task, "dwell_seconds": 3.}})

    def step(self, seconds):
        self.now = 100.+seconds
        self.node.advance_operation(self.now)

    def inspect(self, task="dwell"):
        self.node.core.token = "inspect"
        self.node.core.state = State.INSPECTING
        self.node.begin_effect(self.effect("inspect", "inspect", task=task), self.now)

    def test_buffer_does_not_count_toward_three_seconds(self):
        for t in (0., 1., 4.999):
            self.step(t)
            self.assertIsNone(self.node.still_since)
            self.assertFalse(self.events)
        self.step(5.)
        self.step(7.999)
        self.assertFalse(self.events)
        self.step(8.)
        self.assertEqual(self.events[-1][0], "stopped")
        self.assertEqual(self.node.settle_phase, "verified")

    def test_motion_in_buffer_is_ignored_and_speed_spike_restarts_full_interval(self):
        self.node.stationary = lambda: False
        self.step(4.)
        self.assertFalse(self.pauses)
        self.node.stationary = lambda: True
        self.step(5.)
        self.step(7.5)
        self.node.stationary = lambda: False
        self.step(7.6)
        self.assertEqual(self.node.stationary_resets, 1)
        self.assertFalse(self.pauses)
        self.node.stationary = lambda: True
        self.step(7.7)
        self.step(10.69)
        self.assertFalse(self.events)
        self.step(10.71)
        self.assertEqual(self.events[-1][0], "stopped")

    def test_dwell_uses_verified_interval_without_an_extra_three_seconds(self):
        self.step(5.)
        self.step(8.)
        self.inspect()
        self.step(8.05)
        result = self.events[-1]
        self.assertEqual(result[0], "inspection_succeeded")
        self.assertAlmostEqual(result[3]["stationary_seconds"], 3.05)
        self.assertEqual(result[3]["stationary_start_stamp_ns"], 105_000_000_000)
        self.assertEqual(result[3]["stopped_linear_speed"], .05)

    def test_external_request_waits_and_rechecks_transition_gap(self):
        self.step(5.)
        self.step(8.)
        self.inspect("external")
        self.node.request_pub.publish.assert_not_called()
        self.node.stationary = lambda: False
        self.step(8.05)
        self.node.stationary = lambda: True
        self.step(8.1)
        self.step(11.09)
        self.node.request_pub.publish.assert_not_called()
        self.step(11.11)
        self.node.request_pub.publish.assert_called_once()
        request = json.loads(self.node.request_pub.publish.call_args.args[0].data)
        self.assertEqual(request["schema_version"], 1)
        self.assertEqual(request["goal_stamp_ns"], "1")
        self.assertEqual(request["task_start_stamp_ns"], int(self.now*1e9))
        self.node.stationary = lambda: False
        self.step(11.2)
        self.assertEqual(self.pauses[-1][1], "moved_during_external_task")

    def test_dwell_spike_restarts_instead_of_pausing(self):
        self.step(5.)
        self.step(8.)
        self.inspect()
        self.events.clear()
        self.node.stationary = lambda: False
        self.step(8.05)
        self.node.stationary = lambda: True
        self.step(8.1)
        self.step(11.09)
        self.assertFalse(self.events)
        self.assertFalse(self.pauses)
        self.step(11.11)
        self.assertEqual(self.events[-1][0], "inspection_succeeded")

    def test_new_goal_cancel_and_retry_cannot_reuse_old_timer(self):
        self.step(5.)
        self.step(7.)
        self.node.begin_effect(self.effect("cancel_and_hold", "cancel"), self.now)
        self.assertIsNone(self.node.still_since)
        self.node.begin_effect(self.effect("hold_and_verify", "hold", "P2"), self.now)
        self.step(11.99)
        self.assertIsNone(self.node.still_since)
        self.step(12.)
        self.assertEqual(self.node.still_since, 112.)
        self.assertFalse(self.events)

    def test_leaving_target_pauses_even_in_buffer(self):
        self.node.position_at_target = lambda: False
        self.step(1.)
        self.assertEqual(self.pauses[-1][1], "left_target_during_settle_or_task")
        self.assertFalse(self.events)

    def test_speed_sample_between_timer_ticks_resets(self):
        n = self.node
        self.step(5.)
        n.raw_samples, n.map_samples = deque(), deque()
        n.stamp_fresh = lambda stamp: True
        n.remember = lambda key, msg: n.latest.update({key: msg})
        n.stationary = lambda: False
        msg = Odometry()
        msg.header.stamp.sec = 106
        msg.pose.pose.orientation.w = 1.
        n.odom("raw_odom", msg)
        self.assertIsNone(n.still_since)
        self.assertEqual(n.stationary_resets, 1)
        n.stationary = lambda: True
        self.step(6.05)
        self.step(9.04)
        self.assertFalse(self.events)
        self.step(9.06)
        self.assertEqual(self.events[-1][0], "stopped")

    def test_policy_rejects_impossible_deadline(self):
        for timeout in (5., 8.):
            with self.assertRaises(ValueError):
                Policy(stop_timeout=timeout)
        for key in ("settle_seconds", "stationary_seconds", "stopped_linear_speed"):
            for value in (0., -1., float("nan")):
                with self.assertRaises(ValueError):
                    Policy(**{key: value})


if __name__ == "__main__":
    unittest.main()
