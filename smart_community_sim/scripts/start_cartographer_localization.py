#!/usr/bin/env python3
"""Start a frozen-map trajectory at a known start; reject mismatched scene metadata."""
import hashlib
import json
import math
from pathlib import Path
import rospy
import rospkg
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest


def main():
    rospy.init_node('cartographer_initializer')
    state=Path(rospy.get_param('~state')).expanduser()
    metadata=json.loads(Path(str(state)+'.json').read_text())
    root=Path(rospkg.RosPack().get_path('smart_community_sim'))
    digest=hashlib.sha256((root/'worlds/smart_community.world').read_bytes()).hexdigest()
    if metadata['world_sha256']!=digest:raise ValueError('pbstream belongs to a different world; rebuild it')
    anchor=[float(rospy.get_param('~anchor_'+k)) for k in ('x','y','yaw')]
    if any(abs(a-b)>1e-5 for a,b in zip(anchor,metadata['anchor'])):raise ValueError('Cartographer map anchor does not match saved state')
    pose=[float(rospy.get_param('~'+k)) for k in ('x','y','yaw')]
    dx,dy=pose[0]-anchor[0],pose[1]-anchor[1];c,s=math.cos(anchor[2]),math.sin(anchor[2])
    request=StartTrajectoryRequest()
    request.configuration_directory=str(root/'config/cartographer')
    request.configuration_basename='community_localization.lua'
    request.use_initial_pose=True;request.relative_to_trajectory_id=0
    request.initial_pose.position.x=c*dx+s*dy;request.initial_pose.position.y=-s*dx+c*dy
    request.initial_pose.orientation.z=math.sin((pose[2]-anchor[2])/2)
    request.initial_pose.orientation.w=math.cos((pose[2]-anchor[2])/2)
    rospy.wait_for_service('/start_trajectory',timeout=60)
    response=rospy.ServiceProxy('/start_trajectory',StartTrajectory)(request)
    if response.status.code!=0:raise RuntimeError(response.status.message)
    rospy.loginfo('Cartographer localization trajectory %s initialized',response.trajectory_id)
    rospy.spin()  # Required node must stay alive after initialization.

if __name__=='__main__':main()
