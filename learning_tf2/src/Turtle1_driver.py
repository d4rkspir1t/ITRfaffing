#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
import tf2_ros
import tf_conversions
################# IMPORT all the necessary classes and messages (Twist, Transform message, pose)

def pose_callback(msg):
    global pose                                                                    # Pose needs to be updated throughout the whole script
    pose = msg
    T_1 = TransformStamped()                                                       # We open an empty transform which we fill with information about the pose
    tf_broadcast = tf2_ros.TransformBroadcaster()                                  # We start the broadcaster for the transformations
    # turtlename = rospy.get_param('~turtle')
    T_1.header.stamp = rospy.Time.now()                                            # This is a transformation between world and turtle 1
    T_1.header.frame_id = "world"
    T_1.child_frame_id = "turtle1"
    T_1.transform.translation.x = pose.x
    T_1.transform.translation.y = pose.y
    T_1.transform.translation.z = 0
    
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, pose.theta)     # We use the conversion from Euler to quaternion
    T_1.transform.rotation.x = q[0]
    T_1.transform.rotation.y = q[1]
    T_1.transform.rotation.z = q[2]
    T_1.transform.rotation.w = q[3]

    tf_broadcast.sendTransform(T_1)

def turtle_driver():
    rospy.init_node('circle', anonymous=True)
    sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    twist = Twist()
    pose = Pose()

    while not rospy.is_shutdown(): 
        twist.linear.x = 1
        twist.angular.z= 0.5
        
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        turtle_driver()
    except rospy.ROSInterruptException:
        pass
