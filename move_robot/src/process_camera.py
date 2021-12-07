#!/usr/bin/env python

"""
Reads collected image frames and computes AR tag positions.
Must have the camera node running: `roslaunch lab4_cam run_cam.launch`.
Must have the ar_track node running: `roslaunch lab4_cam ar_track_replay.launch`.
"""

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
import time
import json
import sys
import tf2_ros
import os
from glob import glob
from cv_bridge import CvBridge


def handle_result(base, tags):
    global log
    log = {}
    for tag in tags:
        try:
            pos = tfBuffer.lookup_transform(base, tag, rospy.Time(0))
            nsec = pos.header.stamp.to_nsec()
            tag_poses = log.setdefault(nsec, {})
            tag_poses[tag] = {
                'x': pos.transform.translation.x,
                'y': pos.transform.translation.y,
                'z': pos.transform.translation.z,
                'rx': pos.transform.rotation.x,
                'ry': pos.transform.rotation.y,
                'rz': pos.transform.rotation.z,
                'rw': pos.transform.rotation.w
            }
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)

def publish_images(in_dir, base, tags):
    r = rospy.Rate(5)
    # Publish our target
    for path in glob(os.path.join(in_dir, '*.npy')):
        if rospy.is_shutdown():
            break
        rospy.loginfo("Processing image: {}".format(path))
        rgb_img = np.load(path)
        im_msg = bridge.cv2_to_imgmsg(rgb_img, encoding="rgb8")
        im_msg.header.frame_id = 'usb_cam' # Hardcoded
        im_msg.header.stamp = rospy.Time.now()
        pub.publish(im_msg)
        r.sleep()
        handle_result(base, tags)

            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Publishes camera output at a more controlled rate
    pub = rospy.Publisher('/replay_cam/image_raw', Image, queue_size=10)

    # Converts CV images to sensor_msgs/Image
    bridge = CvBridge()

    # Get data from ar_track_alvar
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    try:
        out_dir = sys.argv[1] if 1 < len(sys.argv) else 'cam_logs/2'
        publish_images(
            out_dir, 
            'ar_marker_6',
            ['ar_marker_0', 'ar_marker_1', 'ar_marker_2', 'ar_marker_3', 'ar_marker_4', 'ar_marker_5']
        )

        # Save log file
        out_file = os.path.join(out_dir, 'processed.json')
        with open(out_file, 'w') as f:
            f.write(json.dumps(log, indent=4))
        print("Recorded tag positions {} times".format(len(log.keys())))
    except rospy.ROSInterruptException:
        pass