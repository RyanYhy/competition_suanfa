#!/usr/bin/env python3
"""Adapted from ~/raicom_ws/src/robot/scripts/keyboard_teleop.py.
Preserves w/x/a/d/s and speed keys; adds mecanum lateral j/l and diagonal q/e.
"""
import select
import sys
import termios
import time
import tty
import rospy
from geometry_msgs.msg import Twist

KEY_MAP={'w':(1,0,0),'x':(-1,0,0),'j':(0,1,0),'l':(0,-1,0),
         'a':(0,0,1),'d':(0,0,-1),'q':(.707,.707,0),
         'e':(.707,-.707,0),'s':(0,0,0),' ':(0,0,0)}

def main():
    rospy.init_node('community_keyboard')
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
    linear=rospy.get_param('~linear_speed',.12)
    angular=rospy.get_param('~angular_speed',.5)
    if not sys.stdin.isatty(): raise RuntimeError('请在交互终端运行键盘节点')
    fd=sys.stdin.fileno();settings=termios.tcgetattr(fd)
    print('w/x 前后 | j/l 左右横移 | a/d 左右转 | q/e 斜移 | s/空格 停止\n'
          'r/f 调线速度 | t/g 调角速度 | p/Ctrl-C 退出。按住按键，松开自动停。',flush=True)
    key='s';last=time.monotonic()
    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            if select.select([sys.stdin],[],[],.05)[0]:
                ch=sys.stdin.read(1)
                if ch in ('p','\x03',''):break
                if ch in KEY_MAP:key=ch;last=time.monotonic()
                elif ch in ('r','f'):linear=max(.03,min(.25,linear+(.02 if ch=='r' else -.02)))
                elif ch in ('t','g'):angular=max(.1,min(1.0,angular+(.1 if ch=='t' else -.1)))
                else:key='s'
            cmd=Twist()
            if time.monotonic()-last<.30:
                vx,vy,w=KEY_MAP[key];cmd.linear.x=vx*linear;cmd.linear.y=vy*linear;cmd.angular.z=w*angular
            pub.publish(cmd)
    finally:
        pub.publish(Twist())
        termios.tcsetattr(fd,termios.TCSADRAIN,settings)

if __name__=='__main__':main()
