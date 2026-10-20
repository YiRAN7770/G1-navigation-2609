"""Actual follower reverse 0.50 m regression, isolated input, no SDK."""
import json
import math
import os
from pathlib import Path as FilePath
import signal
import subprocess
import time
import unittest
import rclpy
from geometry_msgs.msg import PoseStamped,PointStamped,TwistStamped
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool,String

ROOT=FilePath(__file__).resolve().parents[1]
LOG=ROOT/'run_logs/near_goal_20261020/reverse'

class ReverseLimitTest(unittest.TestCase):
    def setUp(self):
        if os.getenv('ROS_LOCALHOST_ONLY')!='1' or int(os.getenv('ROS_DOMAIN_ID','0'))==0:
            raise RuntimeError('Requires isolated localhost ROS domain')
        LOG.mkdir(parents=True,exist_ok=True)
        self.target=None
        rclpy.init();self.n=rclpy.create_node('reverse_limit_fixture')
        self.log=(LOG/(self._testMethodName+'.log')).open('w')
        self.proc=subprocess.Popen([str(ROOT/'ros2_ws/install/local_planner/lib/local_planner/pathFollower'),
             '--ros-args','-p','realRobot:=false','-p','autonomyMode:=true','-p','useFinalGoalPose:=true',
             '-p','goalRequireLocalizationReady:=true','-p','twoWayDrive:='+str(self._testMethodName!='test_one_way_setting_remains_authoritative').lower()],
             stdout=self.log,stderr=subprocess.STDOUT)
        self.addCleanup(self.cleanup)
        self.pubs={k:self.n.create_publisher(ty,tp,10) for k,ty,tp in [
          ('odom',Odometry,'/state_estimation'),('path',Path,'/path'),('point',PointStamped,'/goal_point'),
          ('goal',PoseStamped,'/goal_pose'),('ready',Bool,'/localization_ready'),('joy',Joy,'/joy'),
          ('corridor',Path,'/local_planner/final_path')]}
        self.commands=[];self.statuses=[]
        self.n.create_subscription(TwistStamped,'/cmd_vel',self.commands.append,50)
        self.n.create_subscription(String,'/fastlio_far/goal_status',lambda m:self.statuses.append(json.loads(m.data)),50)
        end=time.monotonic()+6
        while not all(p.get_subscription_count() for p in self.pubs.values()):
            self.spin(.03);self.assertLess(time.monotonic(),end,'DDS discovery timed out')
        self.feed(.2)

    def cleanup(self):
        self.proc.send_signal(signal.SIGINT)
        try:self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
        self.log.close()
        (LOG/(self._testMethodName+'.json')).write_text(json.dumps(self.statuses,indent=2))
        self.n.destroy_node();rclpy.shutdown()

    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self.n,timeout_sec=.004)

    def feed(self,seconds=.35,length=-.8,odom=True,x=0.,yaw=0.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            stamp=self.n.get_clock().now().to_msg()
            if odom:
                o=Odometry();o.header.stamp=stamp;o.header.frame_id='map'
                o.pose.pose.position.x=x
                o.pose.pose.orientation.z=math.sin(yaw/2);o.pose.pose.orientation.w=math.cos(yaw/2)
                self.pubs['odom'].publish(o)
            p=Path();p.header.stamp=stamp;p.header.frame_id='vehicle'
            for v in (0.,length/2,length):
                q=PoseStamped();q.pose.position.x=v;p.poses.append(q)
            self.pubs['path'].publish(p);self.pubs['ready'].publish(Bool(data=True))
            if self.target:
                corridor=Path();corridor.header.stamp=stamp;corridor.header.frame_id='map'
                a=PoseStamped();a.pose.position.x=x;a.pose.orientation.w=1.
                corridor.poses=[a,self.target]
                self.pubs['corridor'].publish(corridor)
            self.spin(.035)
        self.assertIsNone(self.proc.poll())

    def goal(self,x,pose=False,frame='map',yaw=0.,y=0.):
        g=PoseStamped() if pose else PointStamped()
        g.header.stamp=self.n.get_clock().now().to_msg();g.header.frame_id=frame
        if pose:
            g.pose.position.x=x;g.pose.position.y=y;g.pose.orientation.z=math.sin(yaw/2);g.pose.orientation.w=math.cos(yaw/2)
        else:g.point.x=x;g.point.y=y
        self.pubs['goal' if pose else 'point'].publish(g)
        self.target=g if pose else None
        self.spin(.07)
        self.commands.clear()

    def test_inclusive_boundary_and_immediate_revocation(self):
        for distance in (.49999,.5):
            self.goal(-distance);self.feed(1.15)
            self.assertTrue(self.statuses[-1]['reverse_allowed'])
            self.assertTrue(any(m.twist.linear.x<-.02 for m in self.commands))
        self.goal(-.50001);self.feed(.3)
        self.assertFalse(self.statuses[-1]['reverse_allowed'])
        self.assertTrue(self.commands)
        self.assertTrue(all(m.twist.linear.x>=0 for m in self.commands))

    def test_near_local_path_does_not_grant_reverse_for_far_final_goal(self):
        self.goal(-1.,pose=True);self.feed(.4,length=-.07)
        self.assertFalse(self.statuses[-1]['reverse_allowed'])
        self.assertTrue(all(m.twist.linear.x>=0 for m in self.commands))
        self.assertTrue(any(abs(m.twist.angular.z)>.01 for m in self.commands))

    def test_robot_moving_outside_zone_revokes_reverse(self):
        self.goal(-.49);self.feed(.4)
        self.assertTrue(any(m.twist.linear.x<0 for m in self.commands))
        self.feed(.08,x=.02);self.commands.clear();self.feed(.25,x=.02)
        self.assertFalse(self.statuses[-1]['reverse_allowed'])
        self.assertTrue(all(m.twist.linear.x>=0 for m in self.commands))

    def test_distance_is_xy_euclidean(self):
        self.goal(-.4,y=.4);self.feed(.3)
        self.assertFalse(self.statuses[-1]['reverse_allowed'])
        self.assertTrue(all(m.twist.linear.x>=0 for m in self.commands))

    def test_align_inside_zone_is_preserved(self):
        self.goal(-.1,pose=True,yaw=math.pi/2);self.feed(.4)
        self.assertTrue(self.statuses[-1]['reverse_allowed'])
        self.assertEqual(self.statuses[-1]['phase'],'align')
        self.assertFalse(self.statuses[-1]['reached'])
        self.assertTrue(all(m.twist.linear.x==0 for m in self.commands))
        self.assertTrue(any(m.twist.angular.z>0 for m in self.commands))

    def test_invalid_goal_or_stale_pose_revokes_reverse(self):
        self.goal(-.05);self.feed(.4)
        self.assertTrue(self.statuses[-1]['reverse_allowed'])
        self.feed(.65,odom=False);self.commands.clear();self.feed(.2,odom=False)
        self.assertFalse(self.statuses[-1]['reverse_allowed'])
        self.assertTrue(all(m.twist.linear.x>=0 for m in self.commands))
        self.feed(.2);self.goal(-.05,frame='odom');self.feed(.3)
        self.assertFalse(self.statuses[-1]['reverse_allowed'])
        self.assertTrue(all(m.twist.linear.x>=0 for m in self.commands))

    def test_one_way_setting_remains_authoritative(self):
        self.goal(-.05);self.feed(.4)
        self.assertFalse(self.statuses[-1]['reverse_allowed'])
        self.assertTrue(all(m.twist.linear.x>=0 for m in self.commands))

    def test_manual_override_cannot_bypass_distance_limit(self):
        self.goal(-1.)
        joy=Joy();joy.axes=[0.,0.,-1.,0.,-1.,-1.]
        self.pubs['joy'].publish(joy);self.feed(.3)
        self.assertTrue(self.commands)
        self.assertTrue(all(m.twist.linear.x>=0 for m in self.commands))

if __name__=='__main__':unittest.main()
