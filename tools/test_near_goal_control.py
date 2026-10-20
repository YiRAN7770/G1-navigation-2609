"""Real localPlanner + pathFollower on synthetic data; no ROS init on import.

Run only on an isolated localhost ROS domain. No gate, SDK or robot is started.
"""
import json
import math
import os
from pathlib import Path as FilePath
import signal
import subprocess
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String, Header

ROOT = FilePath(__file__).resolve().parents[1]
NAV = ROOT / 'ros2_ws'
LOG = ROOT / 'run_logs/near_goal_20261020/integration'


class NearGoalTest(unittest.TestCase):
    def setUp(self):
        if os.getenv('ROS_LOCALHOST_ONLY') != '1' or int(os.getenv('ROS_DOMAIN_ID', '0')) == 0:
            raise RuntimeError('Requires isolated localhost domain')
        LOG.mkdir(parents=True, exist_ok=True)
        rclpy.init()
        self.node = rclpy.create_node('near_goal_fixture')
        self.procs, self.logs = [], []
        self.addCleanup(self.cleanup)
        for binary, params in [
            ('localPlanner', ['useTerrainAnalysis:=true', 'freezeTime:=0.0',
                              'pathFolder:='+str(NAV/'src/base_autonomy/local_planner/paths')]),
            ('pathFollower', ['realRobot:=false', 'goalRequireLocalizationReady:=true',
                              'useGoalYaw:=false'])]:
            log = (LOG/(self._testMethodName+'_'+binary+'.log')).open('w')
            self.logs.append(log)
            options = ['autonomyMode:=true', 'useFinalGoalPose:=true', 'twoWayDrive:=true'] + params
            args = [str(NAV/'install/local_planner/lib/local_planner'/binary), '--ros-args']
            for value in options:
                args.extend(['-p', value])
            self.procs.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))
        self.pubs = {key: self.node.create_publisher(typ, topic, 10) for key, typ, topic in [
            ('pose', Odometry, '/state_estimation'), ('terrain', PointCloud2, '/terrain_map'),
            ('goal', PoseStamped, '/goal_pose'), ('waypoint', PointStamped, '/way_point'),
            ('ready', Bool, '/localization_ready'), ('stop', Bool, '/fastlio_far/stop')]}
        self.commands, self.states, self.planner, self.paths = [], [], [], []
        self.node.create_subscription(TwistStamped, '/cmd_vel', self.commands.append, 50)
        self.node.create_subscription(String, '/fastlio_far/goal_status',
                                      lambda m: self.states.append(json.loads(m.data)), 50)
        self.node.create_subscription(String, '/local_planner/status',
                                      lambda m: self.planner.append(json.loads(m.data)), 50)
        self.node.create_subscription(Path, '/local_planner/final_path', self.paths.append, 50)
        end = time.monotonic()+12
        while not all(pub.get_subscription_count() for pub in self.pubs.values()):
            self.spin(.05)
            self.assertLess(time.monotonic(), end, 'Discovery timeout')
        self.feed(.25)

    def cleanup(self):
        for proc in self.procs:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for proc in self.procs:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        for log in self.logs:
            log.close()
        if hasattr(self, 'states'):
            (LOG/(self._testMethodName+'.json')).write_text(json.dumps({
                'states': self.states, 'planner': self.planner}, indent=2))
        self.node.destroy_node()
        rclpy.shutdown()

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.003)

    def feed(self, duration=.3, x=0., y=0., yaw=0., obstacles=(), cloud=True,
             odom=True, ready=True, speed=0.):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            stamp = self.node.get_clock().now().to_msg()
            if odom:
                msg = Odometry()
                msg.header.stamp = stamp
                msg.header.frame_id = 'map'
                msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
                msg.pose.pose.orientation.z = math.sin(yaw/2)
                msg.pose.pose.orientation.w = math.cos(yaw/2)
                msg.twist.twist.linear.x = speed
                self.pubs['pose'].publish(msg)
            if cloud:
                fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                          for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
                points = list(obstacles) or [(2., 2., 0., 0.)]
                self.pubs['terrain'].publish(point_cloud2.create_cloud(
                    Header(stamp=stamp, frame_id='map'), fields, points))
            self.pubs['ready'].publish(Bool(data=ready))
            self.spin(.035)
        for proc in self.procs:
            self.assertIsNone(proc.poll())

    def goal(self, x, y=0., yaw=0., waypoint=None):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.pubs['goal'].publish(msg)
        wp = PointStamped()
        wp.header = msg.header
        wp.point.x, wp.point.y = waypoint if waypoint is not None else (float(x), float(y))
        self.pubs['waypoint'].publish(wp)
        self.spin(.08)

    def assertStopped(self):
        self.assertTrue(self.commands)
        self.assertTrue(all(abs(m.twist.linear.x)+abs(m.twist.linear.y)+abs(m.twist.angular.z) < 1e-8
                            for m in self.commands))

    def test_early_align_both_directions_and_boundary(self):
        self.goal(.20001, yaw=math.pi/2)
        self.feed()
        self.assertEqual(self.states[-1]['phase'], 'approach')
        for target_yaw in (math.pi/2, -math.pi/2):
            self.goal(.20, yaw=target_yaw)
            self.feed()
            self.commands.clear()
            self.feed()
            self.assertEqual(self.states[-1]['phase'], 'align')
            self.assertTrue(all(m.twist.linear.x == m.twist.linear.y == 0 for m in self.commands))
            self.assertTrue(all(m.twist.angular.z*target_yaw > 0 for m in self.commands))
            self.assertTrue(all(abs(m.twist.angular.z) <= .15+1e-8 for m in self.commands))

    def test_hold_yaw_lateral_and_reverse_translation(self):
        self.goal(.18, yaw=math.pi/2)
        self.feed(yaw=math.pi/2)
        self.assertEqual(self.states[-1]['phase'], 'translate')
        self.assertAlmostEqual(self.commands[-1].twist.linear.x, 0., places=6)
        self.assertLess(self.commands[-1].twist.linear.y, -.01)
        self.assertAlmostEqual(self.commands[-1].twist.angular.z, 0., places=6)
        self.goal(-.18)
        self.feed()
        self.assertEqual(self.states[-1]['phase'], 'translate')
        self.assertLess(self.commands[-1].twist.linear.x, -.01)
        self.assertAlmostEqual(self.commands[-1].twist.angular.z, 0., places=6)
        self.assertFalse(self.states[-1]['reached'])

    def test_original_goal_overrides_projected_or_behind_waypoint(self):
        self.goal(.24, waypoint=(-.03, .02))
        self.feed(.4)
        self.assertGreater(self.states[-1]['path_points'], 1)
        self.assertEqual(self.planner[-1]['reason'], 'final_corridor_clear')
        self.assertAlmostEqual(self.paths[-1].poses[-1].pose.position.x, .24)
        self.assertGreater(self.commands[-1].twist.linear.x, 0)
        self.goal(-.4)
        self.feed(1.3)
        self.assertTrue(self.states[-1]['reverse_allowed'])
        self.assertGreater(self.states[-1]['path_points'], 1)
        self.assertLess(self.commands[-1].twist.linear.x, 0)
        self.assertNotEqual(self.planner[-1]['reason'], 'waypoint_at_or_behind_robot')
        self.goal(-.74, waypoint=(-.63, .0))
        self.feed(.4)
        self.assertFalse(self.states[-1]['reverse_allowed'])
        self.assertGreater(self.states[-1]['path_points'], 1)
        self.assertGreater(abs(self.commands[-1].twist.angular.z), .01)
        self.assertGreaterEqual(self.commands[-1].twist.linear.x, 0)
        self.assertNotEqual(self.planner[-1]['reason'], 'waypoint_at_or_behind_robot')

    def test_obstacle_revokes_translation_and_alignment(self):
        for target_yaw in (0., math.pi/2):
            self.goal(.18, yaw=target_yaw)
            self.feed(.3, obstacles=[(.25, 0., .2, .5)])
            self.commands.clear()
            self.feed(.3, obstacles=[(.25, 0., .2, .5)])
            self.assertStopped()
            self.assertEqual(self.states[-1]['reason'], 'final_corridor_blocked_or_stale')
            self.assertEqual(len(self.paths[-1].poses), 0)
        self.feed(.4)
        self.assertEqual(self.states[-1]['reason'], 'aligning_yaw')
        self.assertGreater(self.commands[-1].twist.angular.z, 0)

    def test_stale_cloud_and_pose_and_manual_stop(self):
        self.goal(.18)
        self.feed()
        self.feed(.65, cloud=False)
        self.commands.clear()
        self.feed(.2, cloud=False)
        self.assertStopped()
        self.feed(.3)
        self.assertGreater(self.commands[-1].twist.linear.x, 0)
        self.feed(.65, odom=False)
        self.commands.clear()
        self.feed(.2, odom=False)
        self.assertStopped()
        self.feed(.3)
        self.pubs['stop'].publish(Bool(data=True))
        self.feed(.15)
        self.commands.clear()
        self.feed(.2)
        self.assertStopped()
        self.assertEqual(self.states[-1]['reason'], 'manual_stop')

    def test_success_requires_both_tolerances_and_standstill(self):
        self.goal(.10001)
        self.feed()
        self.assertFalse(self.states[-1]['reached'])
        self.assertEqual(self.states[-1]['phase'], 'translate')
        self.goal(.099, yaw=math.radians(21))
        self.feed()
        self.assertFalse(self.states[-1]['reached'])
        self.goal(.099, yaw=math.radians(19))
        self.feed(speed=.06)
        self.assertFalse(self.states[-1]['reached'])
        self.feed()
        self.assertTrue(self.states[-1]['reached'])
        self.goal(.10, yaw=math.radians(20))
        self.feed()
        self.assertTrue(self.states[-1]['reached'])
        self.feed(x=-.02)
        self.assertFalse(self.states[-1]['reached'])
        self.assertGreater(self.states[-1]['distance'], .1)

    def test_heading_hysteresis_and_position_recovery(self):
        self.goal(.18, yaw=math.pi/2)
        self.feed(yaw=math.radians(82))
        self.assertEqual(self.states[-1]['phase'], 'translate')
        self.feed(yaw=math.radians(75))
        self.assertEqual(self.states[-1]['phase'], 'translate')
        self.feed(yaw=math.radians(65))
        self.assertEqual(self.states[-1]['phase'], 'align')
        self.feed(yaw=math.radians(75))
        self.assertEqual(self.states[-1]['phase'], 'align')
        self.feed(yaw=math.radians(82))
        self.assertEqual(self.states[-1]['phase'], 'translate')
        self.feed(x=-.04, yaw=math.radians(82))
        self.assertEqual(self.states[-1]['phase'], 'translate')
        self.feed(x=-.14, yaw=math.radians(82))
        self.assertEqual(self.states[-1]['phase'], 'approach')
        self.assertFalse(self.states[-1]['reached'])

    def test_kinematic_align_then_translate_converges(self):
        self.goal(-.18, .04, yaw=.4)
        x = y = yaw = 0.
        deadline = time.monotonic()+20
        saw_align = saw_translate = False
        while time.monotonic() < deadline:
            start = time.monotonic()
            self.feed(.04, x=x, y=y, yaw=yaw)
            dt = time.monotonic()-start
            state = self.states[-1]
            saw_align |= state['phase'] == 'align'
            saw_translate |= state['phase'] == 'translate'
            if state['reached']:
                break
            cmd = self.commands[-1].twist
            x += (math.cos(yaw)*cmd.linear.x-math.sin(yaw)*cmd.linear.y)*dt
            y += (math.sin(yaw)*cmd.linear.x+math.cos(yaw)*cmd.linear.y)*dt
            yaw += cmd.angular.z*dt
        self.assertTrue(saw_align and saw_translate)
        self.assertTrue(self.states[-1]['reached'])
        self.assertLessEqual(math.hypot(x+.18, y-.04), .1)
        self.assertLessEqual(abs(yaw-.4), math.radians(20))


if __name__ == '__main__':
    unittest.main()
