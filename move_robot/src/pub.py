#!/usr/bin/env python

import rospy
import numpy as np
from iiwa_msgs.msg import JointPosition

# Define the method which contains the node's main functionality
def moveThroughPoints(waypoints, th=1e-2, loop=False):
    global sub, pub, idx
    
    # Current waypoint index
    idx = 0

    # Callback for subscriber to robot JointPosition
    def callback(message):
        global sub, pub, idx
        
        target = np.array(waypoints[idx])
        curr = np.array([message.position.a1, message.position.a2, message.position.a3, message.position.a4,
                        message.position.a5, message.position.a6, message.position.a7])

        # If close enough to target waypoint, move onto the next one or quit
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
        
        # Publish our string to the 'chatter_talk' topic
        pub.publish(pub_joint_position)
        print(curr)
        print(rospy.get_name() + ": I sent \"%s\"" % pub_joint_position.position)

    # Send desired joint positions
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    # Listen to joint states
    sub = rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, callback)
    rospy.spin()

# Define the method which contains the node's main functionality
def talker():

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # iiwa_msgs/JointPosition to the topic /iiwa/command/JointPosition
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    # sub = rospy.Subscriber('/iiwa/state/JointPosition', JointPosition)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(10) # 10hz

    trajectory_waypoints = [[0, 0, 0, 0, 0, 0, 0],
                            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                            [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]]

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        for waypoint in trajectory_waypoints:
            if rospy.is_shutdown():
                break
            
            a1, a2, a3, a4, a5, a6, a7 = waypoint

            # Construct a JointPosition that we want to publish (in Python, the "%"
            # operator functions similarly to sprintf in C or MATLAB)
            pub_joint_position = JointPosition()
            pub_joint_position.header.seq = 0
            # {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ''}
            pub_joint_position.position.a1 = a1
            pub_joint_position.position.a2 = a2
            pub_joint_position.position.a3 = a3
            pub_joint_position.position.a4 = a4
            pub_joint_position.position.a5 = a5
            pub_joint_position.position.a6 = a6
            pub_joint_position.position.a7 = a7
            
            # Publish our string to the 'chatter_talk' topic
            pub.publish(pub_joint_position)
            print(rospy.get_name() + ": I sent \"%s\"" % pub_joint_position.position)
            
            # Use our rate object to sleep until it is time to publish again
            r.sleep()

    # # For list of waypoints
    # trajectory_waypoints = [[0, 0, 0, 0, 0, 0, 0],
    #                         [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
    #                         [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]]
    # for i in range(len(trajectory_waypoints)):
    #      # Construct a JointPosition that we want to publish (in Python, the "%"
    #     # operator functions similarly to sprintf in C or MATLAB)
    #     pub_joint_position = JointPosition()
    #     pub_joint_position.header.seq = 0
    #     pub_joint_position.position.a1 = trajectory_waypoints[i][0]
    #     pub_joint_position.position.a2 = trajectory_waypoints[i][1]
    #     pub_joint_position.position.a3 = trajectory_waypoints[i][2]
    #     pub_joint_position.position.a4 = trajectory_waypoints[i][3]
    #     pub_joint_position.position.a5 = trajectory_waypoints[i][4]
    #     pub_joint_position.position.a6 = trajectory_waypoints[i][5]
    #     pub_joint_position.position.a7 = trajectory_waypoints[i][6]
        
    #     # Publish our string to the 'chatter_talk' topic
    #     pub.publish(pub_joint_position)

    #     print(rospy.get_name() + ": I sent \"%s\"" % pub_joint_position.position)
        
    #     # Use our rate object to sleep until it is time to publish again
    #     r.sleep()

    # # For list of waypoints
    # trajectory_waypoints = # TODO: grab joint angles from simulation at each timestep
    # for i in range(len(trajectory_waypoints)):
    #     pub_joint_position.position.a1 = # trajectory_waypoints[x]
    #     pub_joint_position.position.a2 = # trajectory_waypoints[x]
    #     pub_joint_position.position.a3 = # trajectory_waypoints[x]
    #     pub_joint_position.position.a4 = # trajectory_waypoints[x]
    #     pub_joint_position.position.a5 = # trajectory_waypoints[x]
    #     pub_joint_position.position.a6 = # trajectory_waypoints[x]
    #     pub_joint_position.position.a7 = # trajectory_waypoints[x]
        
    #     # Publish our string to the 'chatter_talk' topic
    #     pub.publish(pub_joint_position)

    #     print(rospy.get_name() + ": I sent \"%s\"" % pub_joint_position.position)
        
    #     # Use our rate object to sleep until it is time to publish again
    #     r.sleep()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        # talker()
        moveThroughPoints([
            [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
            [0, 0, 0, 0, 0, 0, 0]])
    except rospy.ROSInterruptException: pass
