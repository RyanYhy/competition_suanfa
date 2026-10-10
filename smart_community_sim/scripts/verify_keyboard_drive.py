#!/usr/bin/env python3
"""Use a real keyboard node via PTY; judge motion using Gazebo poses and contacts.
Run only with drive_test.launch. Writes a JSON report, exits nonzero on failure.
"""
import json
import csv
import math
import os
import pty
import subprocess
import time
import threading
from pathlib import Path
import rospy
import message_filters
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ContactsState
from tf.transformations import euler_from_quaternion
from smart_community_sim.drive_metrics import compare_motion, stationary_check


class MotionSamples:
    """Exact-stamp truth/encoder pairs, including a CSV trace for each keyboard phase."""
    def __init__(self):
        self.lock=threading.Lock();self.rows=[];self.latest=None;self.phase='settling'
        self.truth=message_filters.Subscriber('/simulation/ground_truth',Odometry)
        self.odom=message_filters.Subscriber('/odom',Odometry)
        self.sync=message_filters.TimeSynchronizer([self.truth,self.odom],100)
        self.sync.registerCallback(self.receive)

    def receive(self,truth,odom):
        def planar(msg):
            p=msg.pose.pose;q=p.orientation
            return (p.position.x,p.position.y,euler_from_quaternion((q.x,q.y,q.z,q.w))[2])
        actual=planar(truth);encoded=planar(odom)
        with self.lock:
            self.latest=(truth.header.stamp.to_sec(),time.monotonic(),actual,encoded)
            self.rows.append(dict(sim_time=truth.header.stamp.to_sec(),phase=self.phase,
                true_x=actual[0],true_y=actual[1],true_yaw=actual[2],
                odom_x=encoded[0],odom_y=encoded[1],odom_yaw=encoded[2],
                true_wz=truth.twist.twist.angular.z,encoder_wz=odom.twist.twist.angular.z))

    def snapshot(self):
        with self.lock:
            if self.latest is not None:
                stamp,received,actual,encoded=self.latest
                if 0<=rospy.Time.now().to_sec()-stamp<=.15 and time.monotonic()-received<1:
                    return actual,encoded
        raise RuntimeError('缺少同步的车体真值/轮式里程计；请重新编译并重启 Gazebo，确认 /simulation/ground_truth 和 /odom 都有数据')

    def save(self,path):
        with self.lock:
            if self.rows:
                with open(path,'w') as stream:
                    writer=csv.DictWriter(stream,fieldnames=list(self.rows[0]));writer.writeheader();writer.writerows(self.rows)


def wait_for_simulation(get, timeout=60):
    """A launch starts Gazebo, spawn_model and this node concurrently."""
    deadline=time.monotonic()+timeout
    initial_clock=rospy.Time.now().to_sec()
    reason='等待 community_mecanum 机器人生成'
    while not rospy.is_shutdown():
        if time.monotonic()>=deadline:
            raise RuntimeError(reason+'超时；检查 Gazebo 启动日志和左下角播放按钮。')
        try:
            state=get('community_mecanum','world')
            if state.success:
                clock=rospy.Time.now().to_sec()
                if clock>0 and clock>initial_clock:
                    return
                reason='等待仿真时钟推进'
        except rospy.ServiceException:
            reason='等待 Gazebo 位姿服务恢复'
        time.sleep(.1)
    raise rospy.ROSInterruptException('验证启动已取消')


def main():
    rospy.init_node('verify_keyboard_drive')
    if rospy.get_param('/community_drive_test',False) is not True:
        raise RuntimeError(
            '当前 ROS 会话不是独立测试场地，验证尚未开始。'
            '请先 Ctrl-C 关闭原来的场地启动程序，再运行：'
            '\nroslaunch smart_community_sim drive_test.launch verify:=true'
            '\n该命令会启动测试场地并自动验证，无需另开 rosrun。')
    rospy.loginfo('等待 Gazebo 服务、机器人生成和仿真时钟就绪……')
    rospy.wait_for_service('/gazebo/get_model_state',timeout=60)
    get=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    wait_for_simulation(get)
    rospy.loginfo('测试场地就绪，开始通过键盘节点验证运动。')
    samples=MotionSamples()
    wheels=set()
    phase_wheels={}
    def contacts(msg):
        for state in msg.states:
            pair=state.collision1_name+' '+state.collision2_name
            if 'field_floor' not in pair:continue
            for name in ('front_left','front_right','rear_left','rear_right'):
                if name+'_roller_' in pair:
                    wheels.add(name);phase_wheels.setdefault(samples.phase,set()).add(name)
    sub=rospy.Subscriber('/ground_contacts',ContactsState,contacts,queue_size=20)
    def pose():
        s=get('community_mecanum','world')
        if not s.success:raise RuntimeError(s.status_message)
        q=s.pose.orientation;r,p,y=euler_from_quaternion((q.x,q.y,q.z,q.w))
        return (s.pose.position.x,s.pose.position.y,s.pose.position.z,r,p,y)
    master,slave=pty.openpty()
    proc=subprocess.Popen(['rosrun','smart_community_sim','keyboard_teleop.py','_linear_speed:=0.12','_angular_speed:=0.5'],stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
    os.close(slave)
    def hold(key,seconds):
        start=rospy.Time.now().to_sec();deadline=time.monotonic()+90
        while rospy.Time.now().to_sec()-start<seconds:
            if proc.poll() is not None:raise RuntimeError('键盘节点提前退出')
            if time.monotonic()>deadline:raise RuntimeError('仿真未推进，检查 Gazebo 是否暂停')
            if key:os.write(master,key.encode())
            time.sleep(.04)
    report=[]
    path=Path(rospy.get_param('~report','/tmp/community_motion_report.json')).expanduser()
    path.parent.mkdir(parents=True,exist_ok=True)
    failure=None
    try:
        hold('s',2)
        cases=[('a','yaw',1),('d','yaw',-1)] if rospy.get_param('~rotation_only',False) else [('w','x',1),('x','x',-1),('j','y',1),('l','y',-1),('a','yaw',1),('d','yaw',-1)]
        for key,axis,sign in cases:
            start=pose();actual_start,odom_start=samples.snapshot();samples.phase=key
            hold(key,2)
            end=pose();actual_end,odom_end=samples.snapshot();samples.phase='braking';hold('s',1)
            dx=end[0]-start[0];dy=end[1]-start[1]
            forward=math.cos(start[5])*dx+math.sin(start[5])*dy
            lateral=-math.sin(start[5])*dx+math.cos(start[5])*dy
            rotation=math.atan2(math.sin(end[5]-start[5]),math.cos(end[5]-start[5]))
            values={'x':forward,'y':lateral,'yaw':rotation}
            value=values[axis]*sign
            stable=abs(end[3])<.20 and abs(end[4])<.20 and abs(end[2]-start[2])<.04
            cross_error=abs(lateral) if axis=='x' else abs(forward)
            correct=value>(.30 if axis=='yaw' else .10)
            if axis!='yaw':correct=correct and cross_error<.10 and abs(rotation)<.25
            else:correct=correct and math.hypot(dx,dy)<.10
            comparison=compare_motion(actual_start,actual_end,odom_start,odom_end)
            contacted=sorted(phase_wheels.get(key,set()))
            row={'key':key,'forward_m':forward,'lateral_m':lateral,'yaw_rad':rotation,'stable':stable,
                 'motion_pass':correct and stable,'ground_contact_wheels':contacted,
                 **comparison,'pass':correct and stable and comparison['odometry_consistent'] and len(contacted)==4}
            report.append(row)
            rospy.loginfo('按键 %s：真实转角 %.1f°，编码器转角 %.1f°，角度误差 %.1f°，通过=%s',
                          key,math.degrees(comparison['true_delta']['yaw_rad']),
                          math.degrees(comparison['odom_delta']['yaw_rad']),math.degrees(comparison['yaw_error_rad']),row['pass'])
        samples.phase='release_stop'
        hold('w',.7);hold(None,1.5);a=pose();hold(None,1);b=pose()
        drift=math.hypot(b[0]-a[0],b[1]-a[1]);turn=abs(math.atan2(math.sin(b[5]-a[5]),math.cos(b[5]-a[5])))
        report.append({'test':'release_stop','drift_m':drift,'yaw_drift_rad':turn,'pass':drift<.025 and turn<.08})
        # A short physical stop can pass while wheel odometry continues drifting.
        # Keep checking both for 10 seconds after the brake transient has settled.
        hold('s',1.5);samples.phase='stationary_hold'
        actual_start,odom_start=samples.snapshot();begin=rospy.Time.now().to_sec()
        hold(None,10)
        actual_end,odom_end=samples.snapshot()
        row=stationary_check(actual_start,actual_end,odom_start,odom_end)
        row['duration_s']=rospy.Time.now().to_sec()-begin;report.append(row)
        rospy.loginfo('静止 10 秒：车体位移 %.3f m / 转角 %.1f°，里程计位移 %.3f m / 转角 %.1f°，通过=%s',
            row['true_drift_m'],math.degrees(row['true_turn_rad']),row['odom_drift_m'],math.degrees(row['odom_turn_rad']),row['pass'])
        report.append({'test':'roller_ground_contacts','wheels':sorted(wheels),'pass':len(wheels)==4})
    except (RuntimeError,rospy.ROSException,OSError) as exc:
        failure=exc;report.append({'test':'execution_error','reason':str(exc),'pass':False})
    finally:
        try:os.write(master,b'p');proc.wait(timeout=3)
        except (OSError,subprocess.TimeoutExpired):proc.terminate()
        os.close(master);sub.unregister()
        with path.open('w') as f:json.dump(report,f,indent=2)
        samples.save(path.with_suffix('.csv'))
    print(json.dumps(report,indent=2));print('Report:',path)
    if failure:raise failure
    if not all(row['pass'] for row in report):raise SystemExit(1)

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except (RuntimeError, rospy.ROSException, OSError) as exc:
        rospy.logerr('%s',exc)
        raise SystemExit(2)
