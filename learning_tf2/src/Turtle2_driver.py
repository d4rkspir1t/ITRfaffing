#!/usr/bin/env python2.7
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import tf_conversions
import numpy as np

threshold = 0.02

def pose_callback(msg):
    global pose
    pose = msg
    T_2 = TransformStamped()
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    T_2.header.stamp = rospy.Time.now()
    T_2.header.frame_id = "world"
    T_2.child_frame_id = "turtle2"
    T_2.transform.translation.x = pose.x
    T_2.transform.translation.y = pose.y
    
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, pose.theta)
    T_2.transform.rotation.x = q[0]
    T_2.transform.rotation.y = q[1]
    T_2.transform.rotation.z = q[2]
    T_2.transform.rotation.w = q[3]

    tf_broadcaster.sendTransform(T_2)

def turtle2_driver_tf():

    rospy.init_node('turtle2_driver', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle2/pose', Pose, pose_callback)
    rate = rospy.Rate(10)

    twist = Twist()

    while not rospy.is_shutdown():
        try:
            T_12 = tf_buffer.lookup_transform('turtle1', 'turtle2', rospy.Time())
            T_21 = tf_buffer.lookup_transform('turtle2', 'turtle1', rospy.Time())

            distance = T_12.transform.translation.x ** 2 + T_12.transform.translation.y ** 2
            angle = np.arctan2(T_12.transform.translation.x, T_12.transform.translation.y)

            # rospy.loginfo(angle)
            if distance > threshold and angle < 0:
                twist.linear.x = 1
                twist.angular.z = 4 * np.arctan2(T_21.transform.translation.y, T_21.transform.translation.x)
                pub.publish(twist)

            elif distance < threshold:
                print("Turtle 2 has caught Turtle 1!")

            else:
                twist.linear.x = 0
        except:
            T_12 = TransformStamped()
            rospy.logerr('cant see shit')
            # continue
        rate.sleep()

if __name__ == '__main__':
    try:
        turtle2_driver_tf()
    except rospy.ROSInterruptException:
        pass