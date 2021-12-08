#!/usr/bin/env python

import rospy
import ros_numpy
from move_robot.srv import Move

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

import numpy as np
import json
import time
from time import sleep
import argparse
import os

# Arguments
def parse():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-w", "--waypoints",
        help="Path to the numpy-serialized joint positions",
        type=str,
        required=True
    )
    parser.add_argument(
        "-c", "--control-freq",
        help="Control frequency for the trajectory - default is 20",
        type=int,
        default=20
    )
    return parser.parse_known_args()

def move_to_initial(move, waypoints):
    rospy.loginfo("Moving to initial position...")
    initial_waypoint = np.repeat(np.load(waypoints)[:1], 100, axis=0)
    np.save('.initial_waypoint', initial_waypoint)
    resp = move('.initial_waypoint.npy', 20)
    rospy.loginfo(resp)

def main(move, waypoints, control_freq):
    # Go to start position and wait
    move_to_initial(move, waypoints)

    rospy.loginfo("Starting...")
    resp = move(waypoints, control_freq)
    rospy.loginfo(resp)

def open_gripper():
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
    command = outputMsg.Robotiq2FGripper_robot_output()
    rospy.loginfo("Opening gripper...")

    while not rospy.is_shutdown():
        command = outputMsg.Robotiq2FGripper_robot_output()

        # activates gripper
        command.rACT = 1

        # go to action
        command.rGTO = 1

        # speed (255 is fastest | 0 is slowest)
        command.rSP  = 255

        # force control (255 for solid, firm objects | 0 for very fragile, deformable objects)
        command.rFR  = 150

        # position request (255 is completely closed | 0 is completely open)
        command.rPR = 0        
        
        pub.publish(command)
        rospy.sleep(0.1)


if __name__ == '__main__':
    # Parse args
    args, _ = parse()

    # Initialize node
    rospy.init_node('move_and_open_gripper', anonymous=True)

    # Call the `move` service
    rospy.wait_for_service('move')
    try:
        move = rospy.ServiceProxy('move', Move)
        main(move, args.waypoints, args.control_freq)
        open_gripper()
    except rospy.ServiceException as e:
        rospy.logerr(e)