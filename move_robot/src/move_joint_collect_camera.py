#!/usr/bin/env python

"""
Moves the robot arm according to some list of waypoints, and
saves each image frame to the given output directory.
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

# Returns a callback that saves the raw image output
def recordIms(out_dir):
    def callback(image):
        name = str(image.header.stamp.to_nsec())
        rgb_image = ros_numpy.numpify(image)
        np.save(os.path.join(out_dir, name), rgb_image)
    return callback

# Define the method which contains the node's main functionality
def moveThroughPoints(waypoints, th=0.07, loop=False, recordFn=None):
    global subs, pub, start_time, idx
    
    # Current waypoint index
    idx = 0
    start_time = None

    # Callback for subscriber to robot JointPosition
    def callback(pos_msg):
        global subs, pub, start_time, end_time, idx

        if not start_time:
            start_time = rospy.Time.now()

        # If close enough to target waypoint, move onto the next one or quit
        pos = pos_msg.position
        target = np.array(waypoints[idx])
        curr = np.array([pos.a1, pos.a2, pos.a3, pos.a4, pos.a5, pos.a6, pos.a7])
        if np.linalg.norm(target - curr) < th:
            idx = idx + 1 if idx + 1 < len(waypoints) else 0
            if idx == 0 and not loop:
                # Done with all the waypoints -> quit
                [sub.unregister() for sub in subs]
                end_time = rospy.Time.now()
                rospy.loginfo('Done!')
                return

        # Publish command
        a1, a2, a3, a4, a5, a6, a7 = waypoints[idx]
        pub_joint_position = JointPosition()
        pub_joint_position.position.a1 = a1
        pub_joint_position.position.a2 = a2
        pub_joint_position.position.a3 = a3
        pub_joint_position.position.a4 = a4
        pub_joint_position.position.a5 = a5
        pub_joint_position.position.a6 = a6
        pub_joint_position.position.a7 = a7
        
        # Publish our target
        pub.publish(pub_joint_position)

    # Send desired joint positions
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    # Listen to joint states and camera output
    subs = [
        rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, callback),
        rospy.Subscriber('/usb_cam/image_raw', Image, recordFn)
    ]

    rospy.spin()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    try:
        pts = np.load('waypoints_small.npy')
        out_dir = sys.argv[1] if 1 < len(sys.argv) else 'cam_logs/0'
        moveThroughPoints(pts, recordFn=recordIms(out_dir))

        # Save start and end time
        out_file = os.path.join(out_dir, 'log.json')
        log = { 'start_time': start_time.to_nsec(), 'end_time': end_time.to_nsec() }
        with open(out_file, 'w') as f:
            f.write(json.dumps(log, indent=4))
    except rospy.ROSInterruptException:
        pass