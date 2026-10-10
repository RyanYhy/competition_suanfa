#!/usr/bin/env python3
"""Measure localization against timestamped simulator truth; never feed truth to control."""
import csv
import hashlib
import json
import math
import threading
import time
from collections import deque
from pathlib import Path
import rospy
import rospkg
import tf
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from smart_community_sim.benchmark_metrics import summarize, wrap, goal_accuracy


def planar(pose):
    q=pose.orientation
    return pose.position.x,pose.position.y,tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2]


class Benchmark:
    def __init__(self):
        self.root=Path(rospkg.RosPack().get_path('smart_community_sim'))
        destination=rospy.get_param('~output_dir','')
        self.output=Path(destination or '/tmp/community_patrol_diagnostics/'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())).expanduser()
        self.output.mkdir(parents=True,exist_ok=True)
        rospy.loginfo('定位/驱动诊断结果目录：%s',self.output)
        if (self.output/'report.json').exists():
            if not rospy.get_param('~overwrite_report',False):raise ValueError('Refusing to overwrite an existing benchmark report')
            for name in ('report.json','trajectory.csv','route_safety.json'):
                path=self.output/name
                if path.exists():path.unlink()
        self.lock=threading.RLock();self.truth=deque(maxlen=3000);self.odom={};self.rows=[];self.goals=[]
        self.phase='startup';self.detail='';self.command=Twist();self.planned=Twist();self.guard='unknown';self.done=False;self.saved=False;self.dropped=0
        self.received=0;self.latest_truth=None;self.started=time.monotonic();self.finished=None
        self.wall_limit=float(rospy.get_param('~wall_timeout',900))
        self.continuous=bool(rospy.get_param('~continuous',False))
        self.listener=tf.TransformListener()
        self.configuration={key:rospy.get_param('~'+key,'') for key in ('localization','planner','profile','amcl_profile','state_file','map_file','keepout_map_file')}
        self.world_hash=hashlib.sha256((self.root/'worlds/smart_community.world').read_bytes()).hexdigest()
        # Evaluation snapshot only. Perception/control never reads this manifest.
        self.scene_manifest=(self.root/'config/scene_manifest.json').read_bytes()
        self.world_snapshot=(self.root/'worlds/smart_community.world').read_bytes()
        self.map_artifacts={}
        for key in ('map_file','keepout_map_file'):
            if not self.configuration[key]:continue
            path=Path(self.configuration[key]).expanduser()
            metadata=yaml.safe_load(path.read_text());pixels=path.parent/metadata['image']
            self.map_artifacts[key]=dict(path=str(path),metadata=metadata,
                yaml_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                image_sha256=hashlib.sha256(pixels.read_bytes()).hexdigest())
        names=['move_base','patrol']+({'amcl':['amcl'],'lidar':['lidar_loc']}.get(self.configuration['localization'],[]))
        self.parameters={name:rospy.get_param('/'+name,{}) for name in names}
        self.route={p['name']:p for p in rospy.get_param('/patrol/waypoints')}
        self.subscribers=[rospy.Subscriber('/simulation/ground_truth',Odometry,self.on_truth,queue_size=100),
                          rospy.Subscriber('/odom',Odometry,self.on_odom,queue_size=100),
                          rospy.Subscriber('/cmd_vel',Twist,self.on_command,queue_size=1),
                          rospy.Subscriber('/navigation/cmd_vel',Twist,self.on_planned,queue_size=1),
                          rospy.Subscriber('/patrol/motion_guard',String,self.on_guard,queue_size=10),
                          rospy.Subscriber('/patrol/status',String,self.on_status,queue_size=20),
                          rospy.Subscriber('/patrol/goal_reached',String,self.on_context,queue_size=20)]
        rospy.on_shutdown(self.save)

    def on_truth(self,msg):
        with self.lock:
            if self.saved:return
            self.received+=1;self.latest_truth=msg
            self.truth.append((msg,time.monotonic(),self.phase,self.detail,self.command,self.planned,self.guard))
    def on_odom(self,msg):
        with self.lock:
            if self.saved:return
            self.odom[msg.header.stamp.to_nsec()]=msg
            if len(self.odom)>4000:
                for key in sorted(self.odom)[:1000]:self.odom.pop(key)
    def on_command(self,msg):
        with self.lock:self.command=msg
    def on_planned(self,msg):
        with self.lock:self.planned=msg
    def on_guard(self,msg):
        with self.lock:self.guard=msg.data
    def on_status(self,msg):
        data=json.loads(msg.data)
        with self.lock:
            if self.continuous and self.done and data['state']=='navigating':
                self.truth.clear();self.odom.clear();self.rows=[];self.goals=[]
                self.received=0;self.dropped=0;self.latest_truth=None
                self.started=time.monotonic();self.finished=None;self.done=False;self.saved=False
            self.phase=data['state'];self.detail=data['detail']
            if self.phase in ('complete','stopped'):
                self.done=True;self.finished=time.monotonic()
    def on_context(self,msg):
        point=json.loads(msg.data).get('name')
        if point not in self.route:return
        with self.lock:
            if self.latest_truth is None:return
            msg=self.latest_truth;x,y,yaw=planar(msg.pose.pose);target=self.route[point]
            self.goals.append(dict(name=point,time=msg.header.stamp.to_sec(),
                position_error_m=math.hypot(x-target['x'],y-target['y']),
                heading_error_deg=math.degrees(abs(wrap(yaw-target['yaw'])))))

    def consume(self):
        with self.lock:
            while self.truth:
                truth,wall,phase,detail,command,planned,guard=self.truth[0]
                stamp=truth.header.stamp;odom=self.odom.get(stamp.to_nsec())
                try:
                    if odom is None:raise ValueError('missing synchronized odometry')
                    # Same simulation timestamp, not an unrelated latest TF pose.
                    position,quaternion=self.listener.lookupTransform('map','base_footprint',stamp)
                    yaw=tf.transformations.euler_from_quaternion(quaternion)[2]
                except (tf.Exception,ValueError):
                    if time.monotonic()-wall<3:return
                    self.truth.popleft();self.dropped+=1;continue
                self.truth.popleft();tx,ty,ta=planar(truth.pose.pose);ox,oy,oa=planar(odom.pose.pose)
                self.rows.append(dict(sim_time=stamp.to_sec(),wall_time=wall-self.started,phase=phase,goal=detail,
                    true_x=tx,true_y=ty,true_yaw=ta,est_x=position[0],est_y=position[1],est_yaw=yaw,
                    odom_x=ox,odom_y=oy,odom_yaw=oa,true_speed=math.hypot(truth.twist.twist.linear.x,truth.twist.twist.linear.y),
                    cmd_speed=math.hypot(command.linear.x,command.linear.y),cmd_yaw=command.angular.z,
                    planned_speed=math.hypot(planned.linear.x,planned.linear.y),
                    odom_speed=math.hypot(odom.twist.twist.linear.x,odom.twist.twist.linear.y),guard=guard))

    def save(self):
        with self.lock:
            if self.saved:return
            self.saved=True
            if self.rows:
                with (self.output/'trajectory.csv').open('w') as stream:
                    writer=csv.DictWriter(stream,fieldnames=list(self.rows[0]));writer.writeheader();writer.writerows(self.rows)
            report=dict(kind='Gazebo live navigation benchmark',state=self.phase,detail=self.detail,
                configuration=self.configuration,parameters=self.parameters,
                world_sha256=self.world_hash,map_artifacts=self.map_artifacts,
                samples_received=self.received,samples_dropped=self.dropped,
                samples_pending=len(self.truth),goals=self.goals,metrics=summarize(self.rows))
            report['passed_route']=self.phase=='complete'
            report.update(goal_accuracy(self.goals,self.route))
            report['sufficient_samples']=len(self.rows)>=100 and (self.dropped+len(self.truth))/max(1,self.received)<.1
            (self.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            (self.output/'scene_manifest.json').write_bytes(self.scene_manifest)
            (self.output/'world.sdf').write_bytes(self.world_snapshot)
            print('Benchmark report: '+str(self.output/'report.json'))

    def run(self):
        while not rospy.is_shutdown():
            if self.continuous and self.saved:
                time.sleep(.05);continue
            self.consume()
            if self.done and time.monotonic()-self.finished>3:
                if not self.continuous:break
                self.save();continue
            if self.received==0 and time.monotonic()-self.started>60:
                self.phase='missing_ground_truth';self.detail='No /simulation/ground_truth: rebuild the drive plugin and restart Gazebo';break
            if time.monotonic()-self.started>self.wall_limit:
                self.phase='timeout';self.detail='benchmark wall-time limit';break
            time.sleep(.05)
        self.save()
        rospy.signal_shutdown('benchmark finished')

if __name__=='__main__':
    rospy.init_node('navigation_benchmark');Benchmark().run()
