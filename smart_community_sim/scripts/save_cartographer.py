#!/usr/bin/env python3
"""Finish mapping and serialize Cartographer with a verified competition-frame anchor."""
import argparse
import hashlib
import json
from pathlib import Path
import rospy
import rospkg
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('state',type=Path)
    args=parser.parse_args(rospy.myargv()[1:]);state=args.state.expanduser().resolve()
    if state.exists() or Path(str(state)+'.json').exists():raise ValueError('Output already exists; select a new filename')
    state.parent.mkdir(parents=True,exist_ok=True)
    rospy.init_node('save_community_cartographer')
    rospy.wait_for_service('/finish_trajectory',timeout=15)
    response=rospy.ServiceProxy('/finish_trajectory',FinishTrajectory)(0)
    if response.status.code!=0:raise RuntimeError(response.status.message)
    rospy.wait_for_service('/write_state',timeout=15)
    response=rospy.ServiceProxy('/write_state',WriteState)(str(state),True)
    if response.status.code!=0:raise RuntimeError(response.status.message)
    root=Path(rospkg.RosPack().get_path('smart_community_sim'))
    metadata=dict(anchor=[rospy.get_param('/cartographer_anchor/'+k) for k in ('x','y','yaw')],
                  world_sha256=hashlib.sha256((root/'worlds/smart_community.world').read_bytes()).hexdigest(),
                  trajectory_id=0,resolution=.02)
    Path(str(state)+'.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print('Saved '+str(state)+' with scene metadata. Mapping trajectory is now finished.')

if __name__=='__main__':main()
