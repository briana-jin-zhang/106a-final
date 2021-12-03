#!/usr/bin/env python

import rospy
import numpy as np
from iiwa_msgs.msg import JointPosition
import time
import json
import sys
import tf2_ros

# Returns a callback that records the position of the given AR tags
def recordTags(base, tags):
    global log, tfBuffer

    log = {}
    def callback(message):
        rospy.loginfo(message)
        for tag in tags:
            try:
                pos = tfBuffer.lookup_transform(base, tag, rospy.Time())
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
    return callback

# Define the method which contains the node's main functionality
def moveThroughPoints(waypoints, th=0.07, loop=False, recordFn=None):
    global sub, pub, tfBuffer, idx
    
    # Current waypoint index
    idx = 0
    start_time = time.time()

    # Callback for subscriber to robot JointPosition
    def callback(message):
        global sub, pub, idx

        # Optional additional callback
        if recordFn:
            recordFn(message)

        # If close enough to target waypoint, move onto the next one or quit
        pos = message.position
        target = np.array(waypoints[idx])
        curr = np.array([pos.a1, pos.a2, pos.a3, pos.a4, pos.a5, pos.a6, pos.a7])
        if np.linalg.norm(target - curr) < th:
            idx += 1
            if idx >= len(waypoints):
                if loop:
                    idx = 0
                else:
                    sub.unregister()
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
        # print(rospy.get_name() + ": I sent \"%s\"" % pub_joint_position.position)

    # Send desired joint positions
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    # Listen to joint states
    sub = rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, callback)
    # Track AR tag positions
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.spin()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    try:
        # pts = np.load('waypoints.npy')
        pts = np.array([
            [-0.85, -36.99, -2.42, 36.47, 3.54, -0.7, -1.49],
            [-0.85, -36.99, -2.42, 36.47, 3.54, -38.16, -1.49],
            [-0.85, -36.99, -2.42, 36.47, 3.54, -0.7, -1.49]
        ]) * np.pi / 180
        moveThroughPoints(pts, recordFn=recordTags(
            'ar_marker_6',
            ['ar_marker_0', 'ar_marker_1', 'ar_marker_2', 'ar_marker_3', 'ar_marker_4', 'ar_marker_5']
        ))
        
        # Save results
        # NOTE: Positions are relative to the base AR tag, not the robot base!
        out_file = sys.argv[1] if 1 < len(sys.argv) else 'log.json'
        with open(out_file, 'w') as f:
            f.write(json.dumps(log, indent=4))
    except rospy.ROSInterruptException:
        pass