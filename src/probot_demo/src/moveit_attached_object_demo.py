#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_attach', anonymous=True)

    arm = moveit_commander.MoveGroupCommander('manipulator')
    end_effector_link = arm.get_end_effector_link()

    arm.set_pose_reference_frame('base_link')
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)
    arm.allow_replanning(True)

    scene = PlanningSceneInterface()
    rospy.sleep(1)

    arm.set_named_target('home')
    arm.go()

    #############################################
    # 移除场景中之前运行残留的物体
    scene.remove_attached_object(end_effector_link, 'tool')
    scene.remove_world_object('table')
    scene.remove_world_object('target')

    # 设置桌面的高度
    table_ground = 0.6
    # 设置table和tool的三维尺寸
    table_size = [0.1, 0.7, 0.01]
    tool_size = [0.2, 0.02, 0.02]

    #############################################
    # 设置tool的位姿
    p = PoseStamped()
    p.header.frame_id = end_effector_link

    p.pose.position.x = tool_size[0] / 2.0 - 0.025
    p.pose.position.y = -0.015
    p.pose.position.z = 0.0
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1

    # 将tool附着到机器人的终端
    scene.attach_box(end_effector_link, 'tool', p, tool_size)

    ############################################
    # 将table加入场景当中
    table_pose = PoseStamped()
    table_pose.header.frame_id = 'base_link'
    table_pose.pose.position.x = 0.25
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = table_ground + table_size[2] / 2.0
    table_pose.pose.orientation.w = 1.0
    scene.add_box('table', table_pose, table_size)

    ############################################
    rospy.sleep(2)
    # 更新当前的位姿
    arm.set_start_state_to_current_state()

    # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
    joint_positions = [0.827228546495185, 0.29496592875743577, 1.1185644936946095, -0.7987583317769674,
                       -0.18950024740190782, 0.11752152218233858]
    arm.set_joint_value_target(joint_positions)

    # 控制机械臂完成运动
    arm.go()
    rospy.sleep(1)

    # 控制机械臂回到初始化位置
    arm.set_named_target('home')
    arm.go()

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()
