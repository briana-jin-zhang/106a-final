#!/usr/bin/env python

import rospy
import sys
import actionlib
from iiwa_msgs.msg import MoveAlongSplineAction, MoveAlongSplineGoal, \
    SplineSegment, CartesianPose


def main():
    rospy.init_node('test_spline_action')

    spline_action_client = actionlib.SimpleActionClient(
        "/iiwa/action/move_along_spline", MoveAlongSplineAction)
    rospy.loginfo("[TestSplineActon] Waiting for server...")
    spline_action_client.wait_for_server()
    rospy.loginfo("[TestSplineActon] ...done!")

    goal = MoveAlongSplineGoal()

    # position: 
    #   x: -0.0661452127851
    #   y: -0.00135995230389
    #   z: 1.30363921576
    # orientation: 
    #   x: 0.00158131697283
    #   y: -0.0421673535053
    #   z: -0.022034506263
    #   w: 0.998866319656

#     poseStamped: 
#   header: 
#     seq: 184666
#     stamp: 
#       secs: 1638499326
#       nsecs: 923000000
#     frame_id: "iiwa_link_0"
#   pose: 
#     position: 
#       x: 0.871740475383
#       y: 0.035432972978
#       z: 0.59648093233
#     orientation: 
#       x: -0.019655938912
#       y: 0.6568103358
#       z: 0.0118461231664
#       w: 0.753706514835
# redundancy: 
#   e1: 0.0286425971546
#   status: 6
#   turn: 120



    seg0 = SplineSegment()
    seg0.type = SplineSegment.LIN
    seg0.point.poseStamped.header.frame_id = 'iiwa_link_0'
    seg0.point.poseStamped.pose.position.x = 0.871740475383
    seg0.point.poseStamped.pose.position.y = 0.035432972978
    seg0.point.poseStamped.pose.position.z = 0.59648093233
    seg0.point.poseStamped.pose.orientation.x = -0.019655938912
    seg0.point.poseStamped.pose.orientation.y = 0.6568103358
    seg0.point.poseStamped.pose.orientation.z = 0.0118461231664
    seg0.point.poseStamped.pose.orientation.w = 0.753706514835
    seg0.point.redundancy.e1 = 0.0286425971546
    seg0.point.redundancy.status = 6
    seg0.point.redundancy.turn = 120

    delta_z = -0.05
    
    seg1 = SplineSegment()
    seg1.type = SplineSegment.LIN
    seg1.point.poseStamped.header.frame_id = 'iiwa_link_0'
    seg1.point.poseStamped.pose.position.x = -0.0661452127851
    seg1.point.poseStamped.pose.position.y = -0.00135995230389
    seg1.point.poseStamped.pose.position.z = 1.30363921576
    seg1.point.poseStamped.pose.orientation.x = 0.00158131697283
    seg1.point.poseStamped.pose.orientation.y = -0.0421673535053
    seg1.point.poseStamped.pose.orientation.z = -0.022034506263
    seg1.point.poseStamped.pose.orientation.w = 0.998866319656
    seg1.point.redundancy.e1 = 0.0286425971546
    seg1.point.redundancy.status = 6
    seg1.point.redundancy.turn = 120

    # goal.spline.segments = [seg0]
    goal.spline.segments = [seg0, seg1]

    rospy.loginfo("[TestSplineAction] Sending spline goal...")
    try:
        spline_action_client.send_goal(goal)
        rospy.loginfo("[TestSplineAction] Waiting for result...")
        spline_action_client.wait_for_result()

        result = spline_action_client.get_result()
        if not result.success:
            rospy.logerr("[TestSplineAction] Failed! Error: {}".format(result.error))
            return False
    except rospy.ROSInterruptException:
        rospy.logerr("[TestSplineAction] Goal interrupted!")
        return False    
    
    rospy.spin()


if __name__ == '__main__':
    main()
