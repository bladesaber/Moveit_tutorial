#!/usr/bin/env python2

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class Detector:
    def __init__(self):
        self.sub = rospy.Subscriber("/probot_anno/camera/image_raw", Image, callback=self.callback)
        self.bridge = CvBridge()

    def callback(self, rgb_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)

        ltop_point, rbottom_point = self.detect_table(rgb_img.copy())
        table_img = rgb_img.copy()[ltop_point[1]:rbottom_point[1], ltop_point[0]:rbottom_point[0], :]

        box_ltop, box_rbottom = self.detect_box(table_img)
        box_ltop = box_ltop+ltop_point
        box_rbottom = box_rbottom + ltop_point

        cv2.rectangle(rgb_img, tuple(ltop_point), tuple(rbottom_point), color=(0,0,255), thickness=2)
        cv2.rectangle(rgb_img, tuple(box_ltop), tuple(box_rbottom), color=(255, 0, 0), thickness=2)
        cv2.imshow('vision', rgb_img)
        cv2.waitKey(1)

    def detect_table(self, rgb_img):
        binary_make = (rgb_img[:, :, 0]<20) & (rgb_img[:, :, 1]<20) & (rgb_img[:, :, 2]<20)

        y_ptr, x_ptr =  np.where(binary_make==1)
        ltop_point = np.array([x_ptr.min(), y_ptr.min()])
        rbottom_point = np.array([x_ptr.max(), y_ptr.max()])

        return ltop_point, rbottom_point

    def detect_box(self, rgb_img):
        binary_make = (rgb_img[:, :, 0] > 150) & (rgb_img[:, :, 1] > 150) & (rgb_img[:, :, 2] > 150)

        y_ptr, x_ptr = np.where(binary_make == 1)
        ltop_point = np.array([x_ptr.min(), y_ptr.min()])
        rbottom_point = np.array([x_ptr.max(), y_ptr.max()])

        return ltop_point, rbottom_point

if __name__ == '__main__':
    rospy.init_node("simple_grasping_vision_detection")

    detector = Detector()

    rospy.spin()
