#!/usr/bin/env python3
"""Check a live launch; no robot controls and no simulation truth feedback."""
import argparse
import json
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    import rospy
    import tf2_ros
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rospy.init_node('community_installation_smoke', anonymous=True, disable_signals=True)
    observations = {}
    errors = []
    subscribers = []

    def receive(topic, msg):
        observations[topic] = msg

    topics = {'/clock': Clock, '/map': OccupancyGrid, '/scan': LaserScan,
              '/odom': Odometry, '/front_camera/image_raw': Image,
              '/front_camera/camera_info': CameraInfo, '/patrol/status': String}
    for topic, message_type in topics.items():
        subscribers.append(rospy.Subscriber(topic, message_type, lambda msg, key=topic: receive(key, msg), queue_size=1))
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    deadline = time.monotonic() + args.timeout
    transform = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            transform = buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        if len(observations) == len(topics) and transform is not None:
            break
        time.sleep(.2)
    for topic in topics:
        if topic not in observations:
            errors.append('No message: ' + topic)
    if transform is None:
        errors.append('Missing TF map -> base_footprint')
    camera = observations.get('/front_camera/image_raw')
    if camera is not None and (camera.width != 1280 or camera.height != 960):
        errors.append('Unexpected camera resolution')
    clock = observations.get('/clock')
    if clock is not None and clock.clock.to_sec() <= 0:
        errors.append('Simulation clock is not advancing')
    if '/scan' in observations and not observations['/scan'].ranges:
        errors.append('Empty laser scan')
    try:
        rospy.wait_for_service('/vision/observation_report', timeout=5)
        response = rospy.ServiceProxy('/vision/observation_report', Trigger)()
        # success=false is normal before any observation frames were requested.
        payload = json.loads(response.message)
        if not isinstance(payload, dict) or 'frames' not in payload:
            errors.append('Invalid vision report response')
    except Exception as exc:
        errors.append('Vision report service: ' + str(exc))
    report = {'scope': 'Live startup, sensors, localization TF and vision service; not a complete navigation acceptance',
              'ok': not errors, 'received_topics': sorted(observations),
              'camera': {'width': camera.width, 'height': camera.height} if camera else None,
              'errors': errors}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
