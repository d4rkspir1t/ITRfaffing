#!/usr/bin/env python3
# coding=utf-8
import rospy
from cv_bridge3 import CvBridge  # Note: normally this would be from cv_bridge import CvBridge
from cv_bridge3 import cv2  # this should be import cv2
import numpy as np
from yolov4 import Detector
import random
from sensor_msgs.msg import Image
from itr_pkg.srv import YOLOLastFrame, YOLOLastFrameResponse
from itr_pkg.msg import YOLODetection
from geometry_msgs.msg import Twist


class YOLOv4ROSITR:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.colors = {}
        self.cam_subs = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback)
        self.yolo_srv = rospy.Service('/detect_frame', YOLOLastFrame, self.yolo_service)
        # self.cam_pub = rospy.Publisher('/itr_pkg/image_raw', Image, queue_size=10)
        self.robot_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/viktor/itr_ws/src/itr_pkg/cfg/coco.data')
        self.srv_resp = None
        self.move_it()

    def move_it(self):
        rate = rospy.Rate(10)  # 10hz
        twist = Twist()

        while not rospy.is_shutdown():
            rospy.wait_for_service('/detect_frame')
            try:
                get_detections = rospy.ServiceProxy('/detect_frame', YOLOLastFrame)
                resp1 = get_detections()
                # rospy.loginfo(resp1)
                self.srv_resp = resp1
            except rospy.ServiceException as e:
                rospy.logerror("Service call failed: %s" % e)

            if self.srv_resp is not None:
                detections = self.srv_resp.detections
                for detection in detections:
                    rospy.loginfo('I\'ve found: %s ' % (detection.name))
                    if 'cup' in detection.name or 'glass' in detection.name:
                        twist.linear.x = 5
                        twist.angular.z = 0
                    elif 'phone' in detection.name:
                        twist.linear.x = 0
                        twist.angular.z = 0.5
                    else:
                        twist = Twist()
                    self.robot_move.publish(twist)
                else:
                    twist = Twist()
                    self.robot_move.publish(twist)
            rate.sleep()

    def yolo_service(self, request):
        res = YOLOLastFrameResponse()
        if self.cv_image is not None:
            cv_copy = self.cv_image.copy()
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)

            cv_height, cv_width, _ = self.cv_image.shape
            # rospy.loginfo(len(detections))
            for detection in detections:
                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)
                # convert bbox to image space
                d.bbox_x = int((d.bbox_x/self.detector.network_width())*cv_width)
                d.bbox_y = int((d.bbox_y/self.detector.network_height())*cv_height)
                d.width = int((d.width/self.detector.network_width())*cv_width)
                d.height = int((d.height/self.detector.network_height())*cv_height)
                res.detections.append(d)
        else:
            rospy.logerror('I have not yet received an image.')
        # return True
        return res

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # rospy.loginfo('got a new image.')


if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()
