"""Watchdog regression: a commanded turn can progress without translation."""
import math
from types import SimpleNamespace
from unittest.mock import Mock
import unittest

from nav_msgs.msg import Odometry
from .core import Effect, Pose
from .ros_node import InspectionNode


class ProgressTest(unittest.TestCase):
    def setUp(self):
        self.node = n = InspectionNode.__new__(InspectionNode)
        n.goal, n.goal_key = Pose(1., 2., 0., .3), "123"
        n.best_error, n.best_heading, n.progress_time = None, None, 0.
        n.fresh = lambda k: True
        n.get_logger = lambda: SimpleNamespace(info=Mock())
        n.latest = {"local": {"goal_stamp_ns": "123", "final_goal_valid": True,
                     "reason": "path_available", "rotation_clear": True,
                     "target_x": 1., "target_y": 2., "target_yaw": .3}, "odom": Odometry()}
        self.nav = {"phase": "approach", "distance": 2., "yaw_error": 3.,
                    "drive_mode": "forward", "raw_wz": .15}
        self.step(0., 3., 0.)

    def step(self, now, heading, yaw, **nav):
        self.nav.update(nav)
        self.node.latest["local"]["waypoint_angle"] = heading
        q = self.node.latest["odom"].pose.pose.orientation
        q.z, q.w = math.sin(yaw/2), math.cos(yaw/2)
        self.node.navigation_progress(self.nav, now)

    def test_long_turn_progresses_without_decreasing_distance(self):
        for t in range(1, 31):
            self.step(float(t), 3.-.07*t, .07*t, distance=2.+t*.005)
            self.assertEqual(self.node.progress_time, t)
        self.assertEqual(self.node.best_error, ("approach", 2.))

    def test_command_without_motion_keeps_timeout(self):
        self.step(16., 3., 0.)
        self.assertEqual(self.node.progress_time, 0.)

    def test_waypoint_change_without_measured_turn_is_not_progress(self):
        self.step(16., 1., 0.)
        self.assertEqual(self.node.progress_time, 0.)

    def test_wrong_or_zero_command_does_not_claim_turn_progress(self):
        for wz in (-.15, 0., float('nan')):
            self.step(16., 2., .8, raw_wz=wz)
            self.assertEqual(self.node.progress_time, 0.)

    def test_motion_away_from_waypoint_is_not_progress(self):
        self.step(16., 2., -.8, raw_wz=-.15)
        self.assertEqual(self.node.progress_time, 0.)

    def test_stale_or_other_goal_observation_is_not_progress(self):
        self.node.fresh = lambda k: k != 'local'
        self.step(16., 2., .8)
        self.assertEqual(self.node.progress_time, 0.)
        self.node.fresh = lambda k: True
        self.node.latest['local']['goal_stamp_ns'] = 'old'
        self.step(17., 2., .8)
        self.assertEqual(self.node.progress_time, 0.)

    def test_oscillation_cannot_repeatedly_reuse_old_improvement(self):
        self.step(1., 2.8, .2)
        self.step(10., 3., 0., raw_wz=-.15)
        self.step(16., 2.8, .2, raw_wz=.15)
        self.assertEqual(self.node.progress_time, 1.)

    def test_angle_wraparound_and_reverse(self):
        self.node.best_heading = (1., 3.13)
        self.step(1., .8, -2.953185307179586)
        self.assertEqual(self.node.progress_time, 1.)
        self.node.best_heading = (-.5, 0.)
        self.step(2., math.pi-.3, -.2, drive_mode='reverse', raw_wz=-.15)
        self.assertEqual(self.node.progress_time, 2.)

    def test_new_goal_resets_turn_history(self):
        n = self.node
        n.zero, n.goal_pub = Mock(), Mock()
        n.operation = None
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: n.latest['odom'].header.stamp))
        n.latest['odom'].header.stamp.sec = 1
        n.core = SimpleNamespace(state=SimpleNamespace(value='NAVIGATING'))
        n.begin_effect(Effect('navigate','new','mission','P2',{'pose':{'x':2.,'y':3.,'z':0.,'yaw':0.}}), 1.)
        self.assertIsNone(n.best_heading)
        self.assertIsNone(n.best_error)


if __name__ == '__main__':
    unittest.main()
