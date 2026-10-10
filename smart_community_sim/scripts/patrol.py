#!/usr/bin/env python3
"""Sequential move_base goals with explicit traffic gates and stop-on-failure."""
import json
import math
import threading
import time
from copy import deepcopy
from pathlib import Path
import actionlib
import rospy
import rospkg
import tf
from nav_msgs.msg import Odometry
from smart_community_sim.traffic_guard import blocked_signal
from smart_community_sim.timing import fresh_sample, limit_velocity
from smart_community_sim.task_progress import LABELS,concise_detail,observation_lines,mission_summary
from smart_community_sim.terminal_log import TerminalLog
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

class TrafficGate:
    def __init__(self):
        self.state=None;self.last_seen=None;self.green_since=None
        self.confirmed_state=None;self.confirmed_seen=None
    def update(self,state,now):
        if state not in ('red','yellow','green'):
            self.state=None;self.green_since=None;self.last_seen=now;return
        if self.last_seen is not None and not 0<=now-self.last_seen<=2.0:
            self.state=None;self.green_since=None
        if state!=self.state:
            recent_phase=(self.confirmed_state in ('red','yellow') and self.confirmed_seen is not None and
                          0<=now-self.confirmed_seen<=2.0)
            self.green_since=now if state=='green' and recent_phase else None
        self.state=state;self.last_seen=now
        self.confirmed_state=state;self.confirmed_seen=now
    def green(self,now):
        return (self.state=='green' and self.last_seen is not None and
                0<=now-self.last_seen<=1.0)
    def ready(self,now):
        # Never treat a first latched green snapshot as a fresh green phase.
        return (self.state=='green' and self.last_seen is not None and
                0<=now-self.last_seen<=2.0 and self.green_since is not None and
                0<=now-self.green_since<=2.0)

class Patrol:
    def __init__(self):
        self.points=rospy.get_param('~waypoints')
        if not self.points:raise ValueError('巡检点列表为空')
        for p in self.points:
            if not all(math.isfinite(float(p[k])) for k in ('x','y','yaw')):raise ValueError('巡检点坐标无效')
        self.frame=rospy.get_param('~frame_id','map')
        root=Path(rospkg.RosPack().get_path('smart_community_sim'))
        self.output=Path(rospy.get_param('~output_dir',str(root/'validation')))
        self.console=TerminalLog(self.output)
        self.counts_cache={};self.point_reports={};self.reached=[]
        self.cancel=threading.Event();self.lock=threading.RLock();self.worker=None
        self.client=actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        self.server_ready=threading.Event()
        threading.Thread(target=self.wait_for_move_base,daemon=True).start()
        self.velocity=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.status=rospy.Publisher('~status',String,queue_size=1,latch=True)
        self.markers=rospy.Publisher('~markers',MarkerArray,queue_size=1,latch=True)
        self.guard_status=rospy.Publisher('~motion_guard',String,queue_size=1,latch=True);self.guard_reason=None
        self.context=rospy.Publisher('~observation_context',String,queue_size=1,latch=True)
        # Observation context is cleared immediately for the next leg. A
        # separate event queue preserves every arrival for passive evaluation.
        self.arrival=rospy.Publisher('~goal_reached',String,queue_size=20,latch=True)
        self.tf_listener=tf.TransformListener()
        self.command=Twist();self.command_received=None;self.command_sim=None
        self.measured=Twist();self.odom_received=None;self.odom_sim=None
        self.wall_freshness=rospy.get_param("~command_wall_timeout",2.0)
        self.require_vision=rospy.get_param("~require_vision",True)
        self.max_translation=float(rospy.get_param("~max_translation",.18))
        self.max_rotation=float(rospy.get_param("~max_rotation",.5))
        self.moving=False
        self.command_sub=rospy.Subscriber('/navigation/cmd_vel',Twist,self.navigation_command,queue_size=1)
        self.odom_sub=rospy.Subscriber('/odom',Odometry,self.odometry,queue_size=1)
        self.counts_sub=rospy.Subscriber('/vision/district_counts',String,self.vision_counts,queue_size=1)
        self.log_sub=rospy.Subscriber('/rosout',Log,self.external_log,queue_size=50)
        prefix=rospy.get_param('~traffic_prefix','/vision/traffic_lights')
        self.gates={name:TrafficGate() for name in {p['signal'] for p in self.points if 'signal' in p}}
        self.subscribers=[rospy.Subscriber(prefix+'/'+name+'/state',String,self.signal,name,queue_size=1) for name in self.gates]
        self.control_timer=rospy.Timer(rospy.Duration(.05),self.forward_command)
        self.start_service=rospy.Service('~start',Trigger,self.start)
        self.stop_service=rospy.Service('~stop',Trigger,self.stop)
        rospy.on_shutdown(self.shutdown)
        self.show_points();self.report('ready','调用 /patrol/start 开始巡检')
        if rospy.get_param('~autostart',False):self.start(None)
    def navigation_command(self,msg):
        with self.lock:
            self.command=Twist()
            self.command.linear.x,self.command.linear.y,self.command.angular.z=limit_velocity(
                msg.linear.x,msg.linear.y,msg.angular.z,self.max_translation,self.max_rotation)
            self.command_received=time.monotonic();self.command_sim=rospy.Time.now().to_sec()
    def vision_counts(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        with self.lock:self.counts_cache=data
    def external_log(self,msg):
        if msg.level>=Log.WARN:self.console.warning(msg.name,msg.msg)
    def odometry(self,msg):
        with self.lock:
            self.measured=msg.twist.twist;self.odom_received=time.monotonic();self.odom_sim=msg.header.stamp.to_sec()
    def forward_command(self,_):
        with self.lock:
            output=Twist()
            sim_now=rospy.Time.now().to_sec();wall_now=time.monotonic()
            command_fresh=fresh_sample(sim_now,self.command_sim,wall_now,self.command_received,wall_timeout=self.wall_freshness)
            odom_fresh=fresh_sample(sim_now,self.odom_sim,wall_now,self.odom_received,wall_timeout=self.wall_freshness)
            fresh=command_fresh and odom_fresh
            reason='idle' if not self.moving or self.cancel.is_set() else ('command_stale' if not command_fresh else ('odom_stale' if not odom_fresh else 'forwarded'))
            if self.moving and not self.cancel.is_set() and fresh:
                try:
                    stamp=self.tf_listener.getLatestCommonTime(self.frame,'base_footprint')
                    if not 0<=(rospy.Time.now()-stamp).to_sec()<.5:raise ValueError('stale localization')
                    pos,quat=self.tf_listener.lookupTransform(self.frame,'base_footprint',rospy.Time(0))
                    yaw=tf.transformations.euler_from_quaternion(quat)[2]
                    now=rospy.Time.now().to_sec()
                    blocked=blocked_signal(pos[0],pos[1],yaw,self.command.linear.x,self.command.linear.y,
                        self.measured.linear.x,self.measured.linear.y,
                        lambda name:name in self.gates and self.gates[name].green(now))
                    if blocked is None:output=self.command
                    else:
                        reason='traffic_stop:'+blocked
                        if reason!=self.guard_reason:
                            self.console.notice('traffic:'+blocked,'停止线保护：等待%s有效绿灯'%{'north_signal':'北侧路口','south_signal':'南侧路口'}.get(blocked,blocked))
                except (tf.Exception,ValueError):
                    reason='localization_stale'
                    rospy.logwarn_throttle(2.0,'定位失联，停止巡检速度输出')
            if reason!=self.guard_reason:
                self.guard_reason=reason;self.guard_status.publish(String(data=reason))
            self.velocity.publish(output)
    def observation(self,point=None):
        if rospy.is_shutdown():return
        point=point or {}
        self.context.publish(String(data=json.dumps({key:point[key] for key in ('district','signal','name') if key in point})))
    def signal(self,msg,name):
        with self.lock:self.gates[name].update(msg.data,rospy.Time.now().to_sec())
    def wait_for_move_base(self):
        # actionlib waits on ROS time; isolate this wait so Stop still works if paused.
        if self.client.wait_for_server():self.server_ready.set()
    def report(self,state,detail):
        if not rospy.is_shutdown():self.status.publish(String(data=json.dumps({'state':state,'detail':detail},ensure_ascii=False)))
        rospy.logdebug('%s: %s',state,detail)
        if state=='complete':self.console.emit('[任务完成] '+concise_detail(detail),color='33')
        elif state=='stopped':self.console.emit('[任务停止] '+concise_detail(detail),color='31')
        elif state=='waiting_signal':self.console.emit(detail)
        elif state=='ready':self.console.emit('巡航就绪，共%d个航点'%len(self.points))
    def show_points(self):
        msg=MarkerArray()
        for i,p in enumerate(self.points):
            m=Marker();m.header.frame_id=self.frame;m.ns='patrol';m.id=i;m.type=Marker.ARROW;m.action=Marker.ADD
            m.pose.position.x=p['x'];m.pose.position.y=p['y'];m.pose.position.z=.04
            m.pose.orientation.z=math.sin(p['yaw']/2);m.pose.orientation.w=math.cos(p['yaw']/2)
            m.scale.x=.18;m.scale.y=.03;m.scale.z=.03;m.color.g=1;m.color.a=1
            msg.markers.append(m)
        self.markers.publish(msg)
    def start(self,_):
        with self.lock:
            if self.worker is not None and self.worker.is_alive():return TriggerResponse(False,'巡检正在运行')
            self.cancel.clear();self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
        return TriggerResponse(True,'巡检已启动；/patrol/status 可查看进度')
    def stop(self,_):
        self.cancel.set()
        with self.lock:self.moving=False;self.velocity.publish(Twist())
        self.client.cancel_all_goals()
        return TriggerResponse(True,'已取消导航目标并停车')
    def shutdown(self):
        self.cancel.set();self.client.cancel_all_goals()
        if not rospy.is_shutdown():self.velocity.publish(Twist())
        if self.worker is not None and self.worker is not threading.current_thread():self.worker.join(timeout=2.)
    def check(self,deadline):
        if self.cancel.is_set() or rospy.is_shutdown():raise RuntimeError('巡检已停止')
        if time.monotonic()>deadline:raise RuntimeError('等待超时，取消目标并停车')
    def dwell(self,seconds):
        start=rospy.Time.now().to_sec();deadline=time.monotonic()+120
        while rospy.Time.now().to_sec()-start<seconds:
            self.check(deadline);self.cancel.wait(.05)
    def wait_green(self,name):
        self.report('waiting_signal',{'north_signal':'北侧路口','south_signal':'南侧路口'}.get(name,name)+'：等待新一轮绿灯')
        self.velocity.publish(Twist());deadline=time.monotonic()+180
        while True:
            self.check(deadline)
            with self.lock:ready=self.gates[name].ready(rospy.Time.now().to_sec())
            if ready:return
            self.cancel.wait(.05)
    def run(self):
        state='stopped';detail='巡航未完成';final_counts=None
        self.console.prepare()
        try:
            with self.lock:self.counts_cache={};self.point_reports={};self.reached=[]
            if self.require_vision:
                rospy.wait_for_service('/vision/reset',timeout=15)
                if not rospy.ServiceProxy('/vision/reset',Trigger)().success:raise RuntimeError('视觉统计初始化失败')
            else:
                from smart_community_sim.run_results import RunResults
                self.navigation_results=RunResults(self.output,{'A':9,'B':9},self.points)
                self.navigation_results.begin({})
            self.console.begin(len(self.points))
            deadline=time.monotonic()+30
            while not self.server_ready.is_set():
                self.check(deadline);self.cancel.wait(.1)
            for index,p in enumerate(self.points,1):
                # Clear attribution while moving; retain a signal through its crossing.
                if not p['name'].endswith('crossing_exit'):self.observation()
                self.check(time.monotonic()+1)
                goal=MoveBaseGoal();goal.target_pose.header.frame_id=self.frame;goal.target_pose.header.stamp=rospy.Time.now()
                goal.target_pose.pose.position.x=p['x'];goal.target_pose.pose.position.y=p['y']
                goal.target_pose.pose.orientation.z=math.sin(p['yaw']/2);goal.target_pose.pose.orientation.w=math.cos(p['yaw']/2)
                self.report('navigating',p['name'])
                with self.lock:
                    self.command=Twist();self.command_received=None;self.command_sim=None;self.moving=True
                self.client.send_goal(goal)
                deadline=time.monotonic()+rospy.get_param('~goal_timeout_wall',120.0)
                while self.client.get_state() in (GoalStatus.PENDING,GoalStatus.ACTIVE,GoalStatus.PREEMPTING,GoalStatus.RECALLING):
                    self.check(deadline);self.cancel.wait(.1)
                if self.client.get_state()!=GoalStatus.SUCCEEDED:raise RuntimeError(p['name']+': '+self.client.get_goal_status_text())
                with self.lock:self.moving=False
                self.velocity.publish(Twist())
                self.reached.append(p['name'])
                self.arrival.publish(String(data=json.dumps(dict(name=p['name'],stamp=rospy.Time.now().to_sec()))))
                self.observation(p)
                self.dwell(float(p.get('dwell',0)))
                if 'signal' in p:self.wait_green(p['signal'])
                if self.require_vision and ('district' in p or 'signal' in p or p['name'].startswith('observe_plate')):
                    response=rospy.ServiceProxy('/vision/observation_report',Trigger)()
                    observation=json.loads(response.message)
                    if observation['observation']!=p['name']:raise RuntimeError('识别报告与当前观察点不一致：'+p['name'])
                    self.point_reports[p['name']]=observation
                    self.console.emit('\n'.join(observation_lines(observation)),color='33')
                self.console.emit('[航点完成 %d/%d] %s'%(index,len(self.points),LABELS.get(p['name'],p['name'])),color='33')
            if self.require_vision:
                result=rospy.ServiceProxy('/vision/finalize',Trigger)()
                final_counts=json.loads(result.message)
                with self.lock:self.counts_cache=final_counts
                if not result.success:raise RuntimeError('路线完成，但视觉覆盖不足：'+result.message)
                detail='单轮路线与识别统计完成：'+result.message
            else:detail='导航诊断路线完成；未执行视觉验收'
            state='complete';self.report(state,detail)
        except (RuntimeError,ValueError,rospy.ROSException,rospy.ServiceException) as exc:
            detail=str(exc)
            if not self.console.active and not self.console.finished:self.console.begin(len(self.points))
            self.report(state,detail)
        finally:
            with self.lock:self.moving=False
            self.observation()
            self.client.cancel_all_goals()
            if not rospy.is_shutdown():self.velocity.publish(Twist())
            with self.lock:
                counts=deepcopy(final_counts if final_counts is not None else self.counts_cache)
                reports=dict(self.point_reports);reports.update(counts.get('observations',{}))
            self.console.finish(mission_summary(self.points,self.reached,counts,reports,state,detail,self.require_vision),complete=state=='complete')
            if not self.require_vision and hasattr(self,'navigation_results'):self.navigation_results.status(state,detail)

if __name__=='__main__':
    rospy.init_node('patrol')
    Patrol();rospy.spin()
