"""Isolated loopback integration of the standalone script with synthetic nav.
No navigation binaries, motion gate, SDK or robot connection is started.
"""
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,String

ROOT=Path(__file__).resolve().parents[1]

class RosRouteTest(unittest.TestCase):
    def setUp(self):
        if os.getenv('ROS_LOCALHOST_ONLY')!='1' or int(os.getenv('ROS_DOMAIN_ID','0'))==0:
            raise RuntimeError('Refusing synthetic nav outside isolated localhost domain')
        self.temp=tempfile.TemporaryDirectory(prefix='single-route-test-')
        self.folder=Path(self.temp.name)
        self.cfg=yaml.safe_load((ROOT/'config/inspection_route.yaml').read_text())
        self.cfg['prior_pcd']=str((ROOT/'config'/self.cfg['prior_pcd']).resolve())
        self.home=dict(x=.3,y=.6,z=.1,yaw=.2)
        self.cfg['points']['START']=self.home.copy()
        self.config=self.folder/'points.yaml';self.config.write_text(yaml.safe_dump(self.cfg))
        rclpy.init();self.executor=SingleThreadedExecutor()
        self.nodes=[rclpy.create_node(name) for name in ('prior_map_localizer','far_planner','pathFollower','route_fixture')]
        for n in self.nodes:self.executor.add_node(n)
        self.nodes[0].declare_parameter('map_path',self.cfg['prior_pcd'])
        self.nodes[1].create_subscription(PoseStamped,'/goal_pose',lambda m:None,10)
        self.nodes[2].create_subscription(PoseStamped,'/goal_pose',self.goal_received,10)
        self.node=self.nodes[-1]
        self.pubs={k:self.node.create_publisher(ty,topic,10) for k,ty,topic in [
            ('odom',Odometry,'/state_estimation'),('raw',Odometry,'/Odometry'),
            ('ready',Bool,'/localization_ready'),('nav',String,'/fastlio_far/goal_status'),
            ('gate',String,'/fastlio_far/motion_status'),('confirmed',Bool,'/goal_reached_confirmed'),
            ('foreign',PoseStamped,'/goal_pose')]}
        self.goals=[];self.stops=[];self.pose=self.home.copy();self.goal=None;self.sent=0.
        self.node.create_subscription(Bool,'/fastlio_far/stop',lambda m:self.stops.append(m.data),10)
        self.proc=None;self.log=None
        self.addCleanup(self.cleanup)

    def cleanup(self):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
        if self.log:self.log.close()
        for n in self.nodes:n.destroy_node()
        self.executor.shutdown();rclpy.shutdown()
        destination=ROOT/'run_logs/independent_patrol/ros'/self._testMethodName
        shutil.copytree(self.folder,destination,dirs_exist_ok=True)
        self.temp.cleanup()

    def launch(self,command):
        self.log=(self.folder/'process.log').open('w')
        self.proc=subprocess.Popen(['python3',str(ROOT/'tools/inspection_route.py'),
           '--config',str(self.config),'--log-dir',str(self.folder/'results')]+command,
           stdout=self.log,stderr=subprocess.STDOUT)

    def goal_received(self,msg):
        self.goal=msg;self.sent=time.monotonic();self.goals.append(msg)

    def feed(self):
        elapsed=time.monotonic()-self.sent
        freeze=self._testMethodName=='test_stall_does_not_skip' and len(self.goals)>=2
        reached=bool(self.goal and elapsed>.25)
        if self.goal and elapsed>.1 and not freeze:
            p,q=self.goal.pose.position,self.goal.pose.orientation
            self.pose=dict(x=p.x,y=p.y,z=p.z,yaw=2*math.atan2(q.z,q.w))
        stamp=self.node.get_clock().now().to_msg()
        for key,frame in [('odom','map'),('raw','camera_init')]:
            o=Odometry();o.header.stamp=stamp;o.header.frame_id=frame
            o.pose.pose.position.x=self.pose['x'];o.pose.pose.position.y=self.pose['y'];o.pose.pose.position.z=self.pose['z']
            o.pose.pose.orientation.z=math.sin(self.pose['yaw']/2);o.pose.pose.orientation.w=math.cos(self.pose['yaw']/2)
            self.pubs[key].publish(o)
        self.pubs['ready'].publish(Bool(data=True));self.pubs['gate'].publish(String(data='ready'))
        self.pubs['confirmed'].publish(Bool(data=reached))
        self.pubs['nav'].publish(String(data=json.dumps(dict(phase='reached' if reached else 'approach' if self.goal else 'idle',
           reason='arrival_latched' if reached else 'tracking',reached=reached))))

    def until(self,predicate,timeout=15):
        end=time.monotonic()+timeout;next_feed=0.
        while time.monotonic()<end:
            if time.monotonic()>=next_feed:self.feed();next_feed=time.monotonic()+.025
            self.executor.spin_once(timeout_sec=.005)
            if predicate():return
        self.fail((self.folder/'process.log').read_text())

    def test_full_single_lap(self):
        self.launch(['run','--dwell','0.25','--timeout','5'])
        self.until(lambda:self.proc.poll() is not None)
        self.assertEqual(self.proc.returncode,0,(self.folder/'process.log').read_text())
        self.assertEqual(len(self.goals),5)
        for msg,name in zip(self.goals,['P1','P2','P3','P4','START']):
            p=self.cfg['points'][name]
            self.assertEqual(msg.pose.position.x,p['x']);self.assertEqual(msg.pose.position.y,p['y'])
            self.assertAlmostEqual(2*math.atan2(msg.pose.orientation.z,msg.pose.orientation.w),p['yaw'])
        self.assertIn(True,self.stops);self.assertNotIn(False,self.stops)
        result=json.loads(next((self.folder/'results').glob('*.json')).read_text())
        self.assertEqual(result['state'],'completed')
        self.assertEqual([p['point'] for p in result['completed']],['P1','P2','P3','P4','START'])
        self.assertEqual(self.node.count_publishers('/cmd_vel_fastlio_far_raw'),0)

    def test_stall_does_not_skip(self):
        self.launch(['run','--dwell','0.25','--timeout','2.5'])
        self.until(lambda:self.proc.poll() is not None)
        self.assertEqual(self.proc.returncode,1)
        self.assertEqual(len(self.goals),2);self.assertIn(True,self.stops)
        self.assertIn('P2 导航超时',(self.folder/'process.log').read_text())

    def test_sigint_stops_without_next_goal(self):
        self.launch(['run'])
        self.until(lambda:len(self.goals)==1)
        self.proc.send_signal(signal.SIGINT)
        self.until(lambda:self.proc.poll() is not None)
        self.assertEqual(self.proc.returncode,130)
        self.assertEqual(len(self.goals),1);self.assertIn(True,self.stops)

    def test_record_only_does_not_publish_goal_or_stop(self):
        self.cfg['points']['START']=None;self.config.write_text(yaml.safe_dump(self.cfg))
        self.launch(['record','START'])
        self.until(lambda:self.proc.poll() is not None)
        self.assertEqual(self.proc.returncode,0,(self.folder/'process.log').read_text())
        self.assertEqual(self.goals,[]);self.assertEqual(self.stops,[])
        saved=yaml.safe_load(self.config.read_text())['points']['START']
        for k,v in self.home.items():self.assertAlmostEqual(saved[k],v)

if __name__=='__main__':unittest.main()
