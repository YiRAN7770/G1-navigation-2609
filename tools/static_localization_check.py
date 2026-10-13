#!/usr/bin/env python3
"""Bounded live sensor/localizer check with the G1 movement backend disabled."""
import argparse,json,os,pathlib,signal,subprocess,time,math
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Imu
from std_msgs.msg import Bool,String
from geometry_msgs.msg import TwistStamped,PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--duration',type=float,default=150);a=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[1];out=pathlib.Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False)
os.environ['ROS_LOG_DIR']=str(out/'ros')
rclpy.init();n=rclpy.create_node('static_localization_verifier');events=open(out/'events.jsonl','w');t0=time.monotonic()
counts={}; stamps={}; first_cloud_time=None; captured=[];capture_saved=False;latest_odom=None;initialized=False;ready_seen=False
summary={'enable_motion':False,'safe_nonzero':0,'diagnostics':{}}
def event(kind,data):events.write(json.dumps({'wall_sec':time.monotonic()-t0,'kind':kind,'data':data})+'\n');events.flush()
def count(topic):
 counts[topic]=counts.get(topic,0)+1;stamps.setdefault(topic,[]).append(time.monotonic()-t0)
def cloud(m):
 global first_cloud_time,capture_saved
 count('/cloud_registered')
 if first_cloud_time is None:first_cloud_time=time.monotonic()
 if not capture_saved:
  captured.append(np.ndarray((m.height,m.width,3),dtype='<f4',buffer=m.data,strides=(m.row_step,m.point_step,4)).reshape(-1,3).copy())
  if time.monotonic()-first_cloud_time>=5:
   np.save(out/'initial_scan.npy',np.concatenate(captured));captured.clear();capture_saved=True;event('scan_saved',True)
def odom(m):
 global latest_odom
 count('/Odometry');latest_odom=m
 event('local_odom',[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])
def ready(m):
 global ready_seen
 count('/localization_ready');ready_seen|=m.data;event('ready',m.data)
def safe(m):
 count('/cmd_vel_fastlio_far_safe');v=m.twist;xyz=[v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]
 summary['safe_nonzero']+=int(any(abs(v)>1e-9 for v in xyz));event('safe',xyz)
def state(m):count('/state_estimation');event('state',[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])
def diag(m):
 for d in m.status:
  summary['diagnostics'][d.message]=summary['diagnostics'].get(d.message,0)+1;event('diagnostic',{'reason':d.message,**{v.key:v.value for v in d.values}})
n.create_subscription(PointCloud2,'/cloud_registered',cloud,qos_profile_sensor_data)
n.create_subscription(Odometry,'/Odometry',odom,qos_profile_sensor_data)
n.create_subscription(Imu,'/imu/data',lambda m:count('/imu/data'),qos_profile_sensor_data)
n.create_subscription(Bool,'/localization_ready',ready,100)
n.create_subscription(TwistStamped,'/cmd_vel_fastlio_far_safe',safe,100)
n.create_subscription(Odometry,'/state_estimation',state,100)
n.create_subscription(DiagnosticArray,'/localization_diagnostics',diag,100)
ip=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10); rp=n.create_publisher(String,'/read_file_dir',10)
script=root/'scripts/run_fastlio_far.sh'
cmd=[str(script),'operation:=localization','planner_mode:=far','map:='+str(root/'maps/site_04_nav_zup_ccw90/prior.pcd'),'enable_motion:=false','start_joy:=false','start_visualization_tools:=false','start_mid360_driver:=true','start_fastlio:=true','network_interface:=eno1','--no-rviz']
summary['command']=cmd;log=open(out/'console.log','w');proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);graph_sent=False
try:
 while time.monotonic()-t0<a.duration:
  if proc.poll() is not None:raise RuntimeError('Launch exited '+str(proc.returncode))
  rclpy.spin_once(n,timeout_sec=.02)
  if not graph_sent and time.monotonic()-t0>12:
   rp.publish(String(data=str(root/'maps/site_04_nav_zup_ccw90/site_04_compact_trajectory.vgh')));graph_sent=True
  seedfile=out/'selected_seed.json'
  if not initialized and latest_odom is not None and seedfile.exists():
   seed=json.loads(seedfile.read_text());y=seed['yaw'];c=math.cos(y);s=math.sin(y);t=seed['translation'];q=latest_odom.pose.pose.orientation;v=latest_odom.pose.pose.position
   msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.pose.pose.position.x=c*v.x-s*v.y+t[0];msg.pose.pose.position.y=s*v.x+c*v.y+t[1];msg.pose.pose.position.z=v.z+t[2]
   sy=math.sin(y/2);cy=math.cos(y/2);r=msg.pose.pose.orientation;r.x=cy*q.x-sy*q.y;r.y=cy*q.y+sy*q.x;r.z=cy*q.z+sy*q.w;r.w=cy*q.w-sy*q.z
   ip.publish(msg);initialized=True;event('initialpose',str(msg))
 summary['nodes']=n.get_node_names();summary['topics']=n.get_topic_names_and_types();summary['ready_seen']=ready_seen;summary['initialized']=initialized
except BaseException as exc:
 summary['error']=repr(exc);raise
finally:
 if proc.poll() is None:proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=15)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 summary['exit']=proc.returncode;summary['counts']=counts
 summary['rates_hz']={k:(len(v)-1)/(v[-1]-v[0]) for k,v in stamps.items() if len(v)>1}
 (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2));events.close();log.close();n.destroy_node();rclpy.try_shutdown()
