#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import action_turtle.msg

def moveClient(x,y):

    client = actionlib.SimpleActionClient('move', action_turtle.msg.moveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    
    # Creates a goal to send to the action server.
    goal = action_turtle.msg.moveGoal([x,y])

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('moveClient')
        result = moveClient(7.5,7.5)
        print("Result:1", result)
        result = moveClient(3.5,7.5)
        print("Result:2", result)
        result = moveClient(3.5,3.5)
        print("Result:3", result)
        result = moveClient(7.5,3.5)
        print("Result:4", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
