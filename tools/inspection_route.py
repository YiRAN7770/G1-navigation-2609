#!/usr/bin/env python3
"""Independent single-lap goal sequencer. No SDK, velocity publisher or nav edits.

show is fully offline. record reads stationary localized pose. run sends one
PoseStamped at a time and finishes at the recorded START. Existing stop latch
is asserted on completion/interruption/failure after sending a goal; never cleared.
"""
import argparse
from collections import deque
from datetime import datetime
import fcntl
import json
import math
import os
import signal
from pathlib import Path
import tempfile
import time

import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/inspection_route.yaml'
SEQUENCE = ['START', 'P1', 'P2', 'P3', 'P4', 'START']
POSITION_TOLERANCE = .10
YAW_TOLERANCE = math.radians(20)


def finite(v):
    return isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v)


def wrap(v):
    return math.atan2(math.sin(v), math.cos(v))


def errors(pose, target):
    return math.hypot(pose['x']-target['x'], pose['y']-target['y']), abs(wrap(pose['yaw']-target['yaw']))


def inside(pose, target):
    distance, angle = errors(pose, target)
    return distance <= POSITION_TOLERANCE and angle <= YAW_TOLERANCE


def load_config(path):
    cfg = yaml.safe_load(Path(path).read_text())
    if not isinstance(cfg, dict) or cfg.get('frame_id') != 'map' or cfg.get('sequence') != SEQUENCE:
        raise ValueError('Expected map frame and START -> P1 -> P2 -> P3 -> P4 -> START')
    if not cfg.get('map_id') or not isinstance(cfg.get('prior_pcd'), str) or not cfg['prior_pcd']:
        raise ValueError('map_id and prior_pcd are required')
    prior = Path(cfg['prior_pcd']).expanduser()
    if not prior.is_absolute():
        prior = Path(path).resolve().parent / prior
    cfg['prior_pcd'] = str(prior.resolve())
    for name in SEQUENCE[:-1]:
        p = cfg.get('points', {}).get(name)
        if name == 'START' and p is None:
            continue
        if not isinstance(p, dict) or not all(finite(p.get(k)) for k in ('x', 'y', 'z', 'yaw')):
            raise ValueError('Invalid pose: '+name)
    return cfg


def atomic_write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix=path.name+'.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(text); f.flush(); os.fsync(f.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp): os.unlink(temp)


def stamp_ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


def decode_pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    if not all(finite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
        raise ValueError('Nonfinite pose')
    norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if abs(norm-1.) > .05:
        raise ValueError('Invalid quaternion')
    return dict(x=p.x, y=p.y, z=p.z,
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y)/norm,1-2*(q.y*q.y+q.z*q.z)/norm))


class Arrival:
    """Old follower lacks goal IDs: require reset -> reached and independent pose.

    The node also rejects competing goals. Three distinct post-goal odometry
    samples and a stable dwell prevent previous-goal or duplicate-frame advance.
    """
    def __init__(self, goal_ns, target, dwell):
        self.goal_ns, self.target, self.dwell = goal_ns, target, dwell
        self.reset_seen = False
        self.samples = deque(maxlen=3)
        self.since = None

    def sample(self, ns, pose):
        if ns > self.goal_ns and (not self.samples or ns > self.samples[-1][0]):
            self.samples.append((ns, pose))

    def check(self, now, nav, stationary):
        good = (self.reset_seen and nav.get('phase') == 'reached'
                and nav.get('reason') == 'arrival_latched' and nav.get('reached') is True
                and len(self.samples) == 3 and all(inside(p, self.target) for _,p in self.samples)
                and stationary)
        if not good:
            self.since = None
            return False
        self.since = now if self.since is None else self.since
        return now-self.since >= self.dwell


def live(args, cfg):
    # These imports do not construct a DDS participant until a live subcommand.
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from rcl_interfaces.srv import GetParameters

    class RouteNode(Node):
        def __init__(self):
            super().__init__('g1_single_route_inspection')
            self.values, self.receipts = {}, {}
            self.raw = deque(maxlen=30)
            self.arrival, self.expected, self.conflict = None, None, False
            self.owned = False
            self.create_subscription(Odometry, '/state_estimation', self.odom, qos_profile_sensor_data)
            self.create_subscription(Odometry, '/Odometry', self.raw_odom, qos_profile_sensor_data)
            self.create_subscription(Bool, '/localization_ready', lambda m:self.remember('ready',m.data),10)
            if args.command == 'run':
                self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
                self.stop_pub = self.create_publisher(Bool, '/fastlio_far/stop', 10)
                self.create_subscription(String, '/fastlio_far/goal_status', self.status, 10)
                self.create_subscription(String, '/fastlio_far/motion_status', lambda m:self.remember('gate',m.data),10)
                self.create_subscription(Bool, '/goal_reached_confirmed', self.confirmed,10)
                self.create_subscription(PoseStamped, '/goal_pose', self.observe,10)
            self.map_client = self.create_client(GetParameters,'/prior_map_localizer/get_parameters')

        def remember(self, key, value):
            self.values[key], self.receipts[key] = value, time.monotonic()

        def fresh(self, key):
            return time.monotonic()-self.receipts.get(key,-math.inf) <= .5

        def valid_stamp(self, msg):
            ns = stamp_ns(msg.header.stamp)
            return ns > 0 and -.1 <= (self.get_clock().now().nanoseconds-ns)/1e9 <= .5

        def odom(self, msg):
            if msg.header.frame_id not in ('map','/map') or not self.valid_stamp(msg): return
            try: p = decode_pose(msg)
            except ValueError: return
            self.remember('pose',p)
            if self.arrival: self.arrival.sample(stamp_ns(msg.header.stamp),p)

        def raw_odom(self, msg):
            if not self.valid_stamp(msg): return
            ns = stamp_ns(msg.header.stamp)
            if self.raw and ns <= self.raw[-1][0]: return
            try: p = decode_pose(msg)
            except ValueError: return
            self.raw.append((ns,p))
            self.remember('raw',msg)

        def stationary(self):
            if not self.fresh('raw') or len(self.raw)<3: return False
            end,p = self.raw[-1]
            window=[(ns,v) for ns,v in self.raw if end-ns <= 800_000_000]
            if len(window)<3 or end-window[0][0]<500_000_000: return False
            for ns,v in window:
                dt=(end-ns)/1e9
                if dt>=.2:
                    distance,angle=errors(p,v)
                    if distance/dt>.03 or angle/dt>.05: return False
            twist=self.values['raw'].twist.twist
            vx,vy,wz=twist.linear.x,twist.linear.y,twist.angular.z
            return all(finite(v) for v in (vx,vy,wz)) and math.hypot(vx,vy)<=.03 and abs(wz)<=.05

        def status(self, msg):
            try: v=json.loads(msg.data)
            except ValueError: return
            if isinstance(v,dict): self.remember('nav',v)

        def confirmed(self, msg):
            if self.arrival and not msg.data: self.arrival.reset_seen=True

        def observe(self, msg):
            if self.expected is not None and msg != self.expected: self.conflict=True

        def healthy(self, motion=False):
            if not self.fresh('pose') or not self.fresh('raw') or not self.fresh('ready') or self.values.get('ready') is not True:
                return False
            topics=['/state_estimation','/Odometry','/localization_ready']
            if motion:
                topics+=['/fastlio_far/goal_status','/goal_reached_confirmed','/fastlio_far/motion_status']
                if not self.fresh('nav') or not self.fresh('gate') or self.values.get('gate')!='ready': return False
            return all(self.count_publishers(t)==1 for t in topics)

        def spin(self): rclpy.spin_once(self,timeout_sec=.03)

        def wait(self, predicate, timeout, message):
            end=time.monotonic()+timeout
            while not predicate():
                if time.monotonic()>end: raise RuntimeError(message)
                self.spin()

        def stop(self):
            if self.owned:
                # Use the existing stop latch; do not generate velocity or clear it.
                for _ in range(5):
                    self.stop_pub.publish(Bool(data=True)); self.spin()

    lock=open('/tmp/g1_single_route_'+os.getenv('ROS_DOMAIN_ID','0')+'.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Keep the ROS context alive while finally publishes the existing stop
    # latch; the default ROS SIGINT handler would shut it down too early.
    previous_term = signal.getsignal(signal.SIGTERM)
    def interrupted(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupted)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node=None; result={'sequence':SEQUENCE,'map_id':cfg['map_id'],'points':cfg['points'],'completed':[]}
    log=Path(args.log_dir)/('route_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    try:
        node=RouteNode()
        node.wait(node.map_client.service_is_ready,10,'定位节点参数服务不可用')
        future=node.map_client.call_async(GetParameters.Request(names=['map_path']))
        node.wait(future.done,10,'地图参数读取超时')
        actual=future.result().values[0].string_value
        if Path(actual).resolve()!=Path(cfg['prior_pcd']).resolve(): raise RuntimeError('当前地图与巡检配置不同')
        node.wait(lambda:node.healthy() and node.stationary(),15,'缺少新鲜定位/原始里程计，或机器人尚未停稳')
        if args.command=='record':
            cfg['points'][args.point]=node.values['pose'].copy()
            atomic_write(args.config,yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True))
            print('已保存',args.point,json.dumps(cfg['points'][args.point],ensure_ascii=False))
            return
        if cfg['points']['START'] is None: raise RuntimeError('请先在实际起点执行 record START')
        if not inside(node.values['pose'],cfg['points']['START']): raise RuntimeError('当前不在已记录起点位姿；请返回起点或重新 record START')
        node.wait(lambda:node.healthy(True) and {'far_planner','pathFollower'} <=
                  {e.node_name for e in node.get_subscriptions_info_by_topic('/goal_pose')}
                  and node.stop_pub.get_subscription_count() > 0,10,
                  '导航未就绪：检查安全门 ready、最终位姿控制及 FAR/pathFollower 订阅者')
        if node.values['nav'].get('phase') not in ('idle','reached'):
            raise RuntimeError('导航仍有未完成目标；请先结束旧任务，不覆盖正在执行的目标')
        result['started_at']=datetime.now().isoformat()
        atomic_write(log,json.dumps(result,ensure_ascii=False,indent=2))
        for name in SEQUENCE[1:]:
            if node.conflict or not node.healthy(True) or not node.stationary():
                raise RuntimeError('目标切换前健康/静止检查失败或目标被覆盖')
            target=cfg['points'][name]
            goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=node.get_clock().now().to_msg()
            goal.pose.position.x=float(target['x']);goal.pose.position.y=float(target['y']);goal.pose.position.z=float(target['z'])
            goal.pose.orientation.z=math.sin(target['yaw']/2);goal.pose.orientation.w=math.cos(target['yaw']/2)
            node.expected=goal;node.arrival=Arrival(stamp_ns(goal.header.stamp),target,args.dwell)
            node.owned=True;node.goal_pub.publish(goal)
            print('发布目标',name,json.dumps(target),flush=True)
            start=time.monotonic();last_print=start-1
            while True:
                node.spin();now=time.monotonic()
                if node.conflict: raise RuntimeError('检测到其他 /goal_pose 来源，终止本轮')
                if not node.healthy(True): raise RuntimeError('定位或导航状态失效，或安全门关闭')
                if not node.arrival.reset_seen and now-start>5:
                    raise RuntimeError('未收到 follower 新目标复位确认；不推进后续点')
                if now-start>args.timeout: raise RuntimeError(name+' 导航超时；不会跳过该点')
                nav=node.values['nav']
                if now-last_print>=1:
                    dist,angle=errors(node.values['pose'],target)
                    print(f'{name}: {nav.get("phase")}/{nav.get("reason")} distance={dist:.3f}m yaw={math.degrees(angle):.1f}deg',flush=True)
                    last_print=now
                if node.arrival.check(now,nav,node.stationary()): break
            dist,angle=errors(node.values['pose'],target)
            result['completed'].append({'point':name,'actual_pose':node.values['pose'].copy(),
                'position_error_m':dist,'yaw_error_deg':math.degrees(angle),'time':datetime.now().isoformat()})
            atomic_write(log,json.dumps(result,ensure_ascii=False,indent=2))
        result['state']='completed'
        print('单轮巡检完成，已返回起点，不自动循环。',flush=True)
    except BaseException as exc:
        result['state']='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed'
        result['reason']=str(exc) or 'Ctrl+C'
        raise
    finally:
        if node:
            try:
                node.stop()
                if args.command=='run': atomic_write(log,json.dumps(result,ensure_ascii=False,indent=2))
            finally:
                node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)
        lock.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('show',help='离线查看位姿和顺序，不发布任何消息')
    record=sub.add_parser('record',help='读取并保存当前静止定位位姿')
    record.add_argument('point',choices=SEQUENCE[:-1])
    run=sub.add_parser('run',help='按顺序发布单轮巡检任务；该命令可能驱动实机')
    run.add_argument('--timeout',type=float,default=180.,help='每段超时秒数')
    run.add_argument('--dwell',type=float,default=3.,help='到点后连续确认停稳秒数')
    parser.add_argument('--log-dir',type=Path,default=DEFAULT_CONFIG.parents[1]/'run_logs/single_route')
    args=parser.parse_args()
    cfg=load_config(args.config)
    if args.command=='show':
        print(yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True));return
    if args.command=='run' and any(not finite(v) or v<=0 for v in (args.timeout,args.dwell)):
        parser.error('timeout 和 dwell 必须为有限正数')
    try: live(args,cfg)
    except KeyboardInterrupt: print('巡检已中断，已尝试锁停。');raise SystemExit(130)
    except Exception as exc: print('未完成：'+str(exc));raise SystemExit(1)


if __name__=='__main__':main()
