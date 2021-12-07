#!/usr/bin/env python

"""
Moves the robot arm according to some
trajectory and publishes status updates.
"""

import rospy
from iiwa_msgs.msg import JointPosition
from move_robot.srv import Move, MoveResponse

import numpy as np

# Move through the given joint positions
def move(req):
    # Get arguments
    pts = np.load(req.waypoints)
    control_freq = req.control_freq

    # Send desired joint positions at the given frequency
    move_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    r = rospy.Rate(control_freq)

    rospy.loginfo("Starting...")
    start_nsec = rospy.Time.now().to_nsec()
    for pt in pts:
        if rospy.is_shutdown():
            break

        a1, a2, a3, a4, a5, a6, a7 = pt
        joint_position = JointPosition()
        joint_position.position.a1 = a1
        joint_position.position.a2 = a2
        joint_position.position.a3 = a3
        joint_position.position.a4 = a4
        joint_position.position.a5 = a5
        joint_position.position.a6 = a6
        joint_position.position.a7 = a7

        move_pub.publish(joint_position)
        r.sleep()
    end_nsec = rospy.Time.now().to_nsec()
    rospy.loginfo("Done!")

    resp = MoveResponse()
    resp.status = "Done"
    resp.start_nsec = start_nsec
    resp.end_nsec = end_nsec
    return resp

if __name__ == '__main__':
    # Initialize service
    rospy.init_node('move_server')
    s = rospy.Service('move', Move, move)
    rospy.loginfo("Move service started.")
    rospy.spin()