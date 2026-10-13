"""Boundary checks of the production stop predicate; no ROS graph or robot."""
from types import SimpleNamespace
import unittest

from nav_msgs.msg import Odometry
from .core import Policy, Pose
from .ros_node import InspectionNode


class StationaryTest(unittest.TestCase):
    def sample(self, measured=0., twist=0., angular=0.):
        odom = Odometry()
        odom.twist.twist.linear.x = twist
        odom.twist.twist.angular.z = angular
        return SimpleNamespace(
            fresh=lambda key: True,
            core=SimpleNamespace(policy=Policy()),
            latest={"odom": odom, "raw_odom": odom},
            raw_samples=[(t, Pose(measured*t/1e9, 0., 0., 0.))
                         for t in (0, 200_000_000, 400_000_000, 600_000_000)])

    def test_twist_boundary_and_angular_limit(self):
        for speed, accepted in ((.049, True), (.05, False), (.051, False)):
            with self.subTest(speed=speed):
                self.assertEqual(InspectionNode.stationary(self.sample(twist=speed)), accepted)
        self.assertFalse(InspectionNode.stationary(self.sample(angular=.051)))

    def test_measured_motion_even_with_zero_twist(self):
        self.assertTrue(InspectionNode.stationary(self.sample(measured=.049)))
        self.assertFalse(InspectionNode.stationary(self.sample(measured=.051)))

    def test_fresh_sustained_window_still_required(self):
        sample = self.sample()
        sample.raw_samples = sample.raw_samples[-2:]
        self.assertFalse(InspectionNode.stationary(sample))
        sample = self.sample()
        sample.fresh = lambda key: False
        self.assertFalse(InspectionNode.stationary(sample))
