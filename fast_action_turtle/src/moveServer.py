#! /usr/bin/env python

import rospy

import actionlib

import action_turtle.msg

import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from std_msgs.msg import String


class moveAction(object):
    # create messages that are used to publish feedback/result
    _feedback = action_turtle.msg.moveFeedback()
    _result = action_turtle.msg.moveResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action_turtle.msg.moveAction, execute_cb=self.execute_cb, auto_start = False)
        
        self.currentRotation = 0
        self.positionGoal = 0
        self.goal = 0
        
        self._as.start()

    def callback(self, data):
        self.currentPosition = [data.x, data.y]
        self.currentRotation = data.theta
        a = self.currentPosition
        b = self.goal.goal
        rospy.loginfo('Turtle position: ('+ str(data.x) + ', ' + str(data.y) + ') ')

    def execute_cb(self, goal):
        self.goal = goal

        r = rospy.Rate(10)
        success = True

        topicPub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        topicSub = rospy.Subscriber('/turtle1/pose', Pose, self.callback)

        b = self.goal.goal
        
        rospy.loginfo('coordinatesToReach: ' + str(b[0]) + ',' + str(b[1]))
        
        
        sleepTime = 2.5

        rospy.sleep(sleepTime)
        r.sleep()

        a = self.currentPosition
        b = self.goal.goal
        
        #Rotate to the right direction
        twist = Twist()
        twist.angular.z = math.atan2(b[1] - a[1], b[0] - a[0])
        topicPub.publish(twist)
        rospy.sleep(sleepTime)
        
        r.sleep()
        
        #Go forward
        twist = Twist()
        twist.linear.x = math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
        topicPub.publish(twist)
        rospy.sleep(sleepTime)
        
        r.sleep()
        
        #Go back to starting rotation, this is to process next coordinates
        twist = Twist()
        twist.angular.z = - self.currentRotation
        topicPub.publish(twist)
        rospy.sleep(sleepTime)
        
        r.sleep()      

        if success:
            #self._result = "Success!"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('move')
    server = moveAction(rospy.get_name())
    rospy.spin()
