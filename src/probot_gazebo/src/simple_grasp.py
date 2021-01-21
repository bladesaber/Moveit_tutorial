#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import cv2
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Point
import tf
import random
import time

class Detector:
    def __init__(self):
        self.sub = rospy.Subscriber("/probot_anno/camera/image_raw", Image, callback=self.callback)
        self.bridge = CvBridge()

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("/base_link", "/camera_link", rospy.Time(), rospy.Duration(4))

        got_tf = False
        while not got_tf:
            try:  # 如果查询得到的话，就将结果保存到camera_to_robot_，保存x,y,z和四元数一共7个值
                (self.trans, self.rot) = self.tf_listener.lookupTransform("/base_link", "/camera_link", rospy.Time(0))
                rospy.loginfo("[adventure_tf]: got")
                got_tf = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("[adventure_tf]: (wait)")
                rospy.sleep(1.0)

        self.box_center_z = None
        self.box_center_y = None
        self.box_center_z = None

    def callback(self, rgb_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)

        ltop_point, rbottom_point = self.detect_table(rgb_img.copy())
        table_img = rgb_img.copy()[ltop_point[1]:rbottom_point[1], ltop_point[0]:rbottom_point[0], :]

        box_ltop, box_rbottom = self.detect_box(table_img)
        box_ltop = box_ltop+ltop_point
        box_rbottom = box_rbottom + ltop_point
        box_center = (box_ltop+box_rbottom)/2.0
        box_center = box_center.astype(np.int)

        cv2.rectangle(rgb_img, tuple(ltop_point), tuple(rbottom_point), color=(0,0,255), thickness=2)
        cv2.rectangle(rgb_img, tuple(box_ltop), tuple(box_rbottom), color=(255, 0, 0), thickness=2)
        cv2.circle(rgb_img, (box_center[0], box_center[1]), 2, (255, 0, 0))
        cv2.imshow('vision', rgb_img)
        cv2.waitKey(1)

        obj_camera_frame  = PointStamped(
            header=rospy.Header(stamp = rospy.get_rostime(), frame_id="/camera_link"),
            point=Point(
                x=0.45,
                y=-box_center[1],
                z=-box_center[0]))
        obj_robot_frame = self.tf_listener.transformPoint(target_frame="/base_link", ps=obj_camera_frame)
        print("obj_robot_frame_tf", obj_robot_frame)

        self.box_center_x = obj_robot_frame.point.x
        self.box_center_y = obj_robot_frame.point.y
        self.box_center_z = obj_robot_frame.point.z

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

def attainPosition(x, y, z, move_group):
    current_pose = move_group.get_current_pose()

    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    target_pose.pose.orientation = current_pose.pose.orientation

    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(target_pose, end_effector_link)
    move_group.go()

def attainObject():
    if vision_manager.box_center_y is not None:
        attainPosition(vision_manager.box_center_x,
                       vision_manager.box_center_y,
                       vision_manager.box_center_z,
                       arm)

        grasper.set_named_target('open')
        grasper.go()

        curent_pose = arm.get_current_pose()
        z = curent_pose.pose.position.z
        z = z - 0.02
        attainPosition(curent_pose.pose.position.x,
                       curent_pose.pose.position.y,
                       z,
                       arm)

        grasper.set_named_target('close')
        grasper.go()

        curent_pose = arm.get_current_pose()
        y = curent_pose.pose.position.y
        if random.random()>0.5:
            y = y + 0.02
        else:
            y = y- 0.02
        attainPosition(curent_pose.pose.position.x,
                       y,
                       curent_pose.pose.position.z,
                       arm)

        grasper.set_named_target('open')
        grasper.go()

        curent_pose = arm.get_current_pose()
        z = curent_pose.pose.position.z
        z = z + 0.02
        attainPosition(curent_pose.pose.position.x,
                       curent_pose.pose.position.y,
                       z,
                       arm)

        arm.set_named_target('home')
        arm.go()
    else:
        print 'can not find object or tf'

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("simple_grasp", anonymous=True)

    arm = moveit_commander.MoveGroupCommander('manipulator')
    end_effector_link = arm.get_end_effector_link()

    arm.set_goal_joint_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)

    grasper = moveit_commander.MoveGroupCommander('gripper')

    vision_manager = Detector()

    length = 0.3
    breadth = 0.3
    pregrasp_x = 0.20
    pregrasp_y = -0.17
    pregrasp_z = 0.28

    arm.set_named_target('home')
    arm.go()

    while not rospy.is_shutdown():
        attainPosition(pregrasp_x, pregrasp_y, pregrasp_z, arm)
        attainObject()
        time.sleep(2)