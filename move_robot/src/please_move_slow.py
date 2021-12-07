#!/usr/bin/env python

"""
Moves the robot arm slowly and smoothly between two points
"""

import rospy
import ros_numpy
import numpy as np
from iiwa_msgs.msg import JointPosition
from sensor_msgs.msg import Image
import message_filters
import time
import json
import sys
import tf2_ros
import os

# Returns a callback that records the position of the given AR tags
def recordTags(base, tags):
    global log, tfBuffer

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

# Define the method which contains the node's main functionality
def moveToPoint(start, end, duration=5000):
    # Send desired joint positions
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)

    nsteps = duration / 2
    dq = (end - start) / nsteps
    r = rospy.Rate(500)
    pos = start
    steps = 0
    while not rospy.is_shutdown():
        if steps >= nsteps:
            return
        steps += 1
        pos += dq

        recordTags(
            'ar_marker_6',
            ['ar_marker_0', 'ar_marker_1', 'ar_marker_2', 'ar_marker_3', 'ar_marker_4', 'ar_marker_5']
        )

        a1, a2, a3, a4, a5, a6, a7 = pos
        pub_joint_position = JointPosition()
        pub_joint_position.position.a1 = a1
        pub_joint_position.position.a2 = a2
        pub_joint_position.position.a3 = a3
        pub_joint_position.position.a4 = a4
        pub_joint_position.position.a5 = a5
        pub_joint_position.position.a6 = a6
        pub_joint_position.position.a7 = a7

        pub.publish(pub_joint_position)
        r.sleep()

def move_waypoints(pts, segment_duration=5000):
    global tfBuffer

    # Track AR tag positions
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    for i in range(len(pts) - 1):
        start, end = pts[i], pts[i + 1]
        moveToPoint(start, end, duration=segment_duration)
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Logs AR tag positions
    log = {}

    try:
        # pts = np.load('waypoints.npy')
        start_6 = -0.7
        end_6 = -38.16
        joint_angles_start = [-0.85, -36.99, -2.42, 36.47, 3.54, start_6, -1.49]
        joint_angles_end = [-0.85, -36.99, -2.42, 36.47, 3.54, end_6, -1.49]
        pts = np.array([
            joint_angles_start,
            joint_angles_end,
            joint_angles_start
        ]) * np.pi / 180
        move_waypoints(pts, segment_duration=5000)

        out_file = sys.argv[1] if 1 < len(sys.argv) else 'log.json'
        with open(out_file, 'w') as f:
            f.write(json.dumps(log, indent=4))
        print("Recorded tag positions {} times".format(len(log.keys())))
    except rospy.ROSInterruptException:
        pass