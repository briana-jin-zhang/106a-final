#!/usr/bin/env python

import rospy
import ros_numpy
from move_robot.srv import Move
from std_msgs.msg import String
from sensor_msgs.msg import Image
from iiwa_msgs.msg import CartesianPose

import numpy as np
import json
import time
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
    parser.add_argument(
        "-o", "--out-dir",
        help="Output directory to store log files",
        type=str,
        default="output"
    )
    parser.add_argument(
        "--log-camera",
        help="Flag for collecting raw camera output",
        action='store_true'
    )
    parser.add_argument(
        "--log-pose",
        help="Flag for collecting end-effector's cartesian pose",
        action='store_true'
    )
    return parser.parse_known_args()

def collect_camera(out_dir):
    rospy.loginfo("Collecting camera output")
    cam_out_dir = os.path.join(out_dir, 'camera')
    os.mkdir(cam_out_dir)
    def callback(image):
        name = str(image.header.stamp.to_nsec())
        rgb_image = ros_numpy.numpify(image)
        np.save(os.path.join(cam_out_dir, name), rgb_image)
    return rospy.Subscriber('/usb_cam/image_raw', Image, callback)

# NOTE: Subscribing to `/iiwa/state/CartesianPose` will take up all the
# network bandwidth, resulting in choppy movements.
def collect_pose(out_dir):
    rospy.loginfo("Collecting end-effector pose")
    pose_out_dir = os.path.join(out_dir, 'pose')
    os.mkdir(pose_out_dir)
    def callback(msg):
        pose = msg.poseStamped
        stamp_nsec = pose.header.stamp.to_nsec()
        with open(os.path.join(pose_out_dir, '{}.json'.format(stamp_nsec)), 'w') as f:
            f.write(json.dumps({
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z,
                'rx': pose.pose.orientation.x,
                'ry': pose.pose.orientation.y,
                'rz': pose.pose.orientation.z,
                'rw': pose.pose.orientation.w
            }, indent=4))
    return rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, callback)

def move_to_initial(move, waypoints):
    rospy.loginfo("Moving to initial position...")
    initial_waypoint = np.repeat(np.load(waypoints)[:1], 100, axis=0)
    np.save('.initial_waypoint', initial_waypoint)
    resp = move('.initial_waypoint.npy', 20)
    rospy.loginfo(resp)

def main(move, waypoints, control_freq, out_dir, collect=[]):
    # Go to start position and wait
    move_to_initial(move, waypoints)

    rospy.loginfo("Starting...")
    subs = [fn(out_dir) for fn in collect]
    resp = move(waypoints, control_freq)
    [sub.unregister for sub in subs]
    rospy.loginfo(resp)

    # Save response to json file
    if not collect:
        return
    with open(os.path.join(out_dir, 'status.json'), 'w') as f:
        f.write(json.dumps({
            'status': resp.status,
            'start_nsec': resp.start_nsec,
            'end_nsec': resp.end_nsec
        }, indent=4))

if __name__ == '__main__':
    # Parse args
    args, _ = parse()

    # Data to collect
    collect = []
    if args.log_camera:
        collect.append(collect_camera)
    if args.log_pose:
        collect.append(collect_pose)

    # Create output directory
    if collect:
        os.makedirs(args.out_dir)

    # Initialize node
    rospy.init_node('move_and_collect', anonymous=True)

    # Call the `move` service
    rospy.wait_for_service('move')
    try:
        move = rospy.ServiceProxy('move', Move)
        main(move, args.waypoints, args.control_freq, args.out_dir, collect=collect)
    except rospy.ServiceException as e:
        rospy.logerr(e)