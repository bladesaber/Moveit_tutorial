#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from copy import deepcopy
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_cartesian_demo')

    arm = moveit_commander.MoveGroupCommander('manipulator')
    end_effector_link = arm.get_end_effector_link()

    arm.set_max_acceleration_scaling_factor(0.8)
    arm.set_max_velocity_scaling_factor(0.8)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    arm.set_pose_reference_frame('base_link')
    arm.allow_replanning(True)

    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = 0.331958
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.307887
    target_pose.pose.orientation.x = -0.482974
    target_pose.pose.orientation.y = -0.482974
    target_pose.pose.orientation.z = -0.504953
    target_pose.pose.orientation.w = -0.494393

    arm.set_pose_target(target_pose, end_effector_link)

    traj = arm.plan()
    arm.execute(traj)
    rospy.sleep(1)

    center_a = target_pose.pose.position.y
    center_b = target_pose.pose.position.z
    radius = 0.1

    waypoints = []
    start_pose = deepcopy(target_pose.pose)
    step = 100

    # y_list, z_list = [], []
    waypoints.append(start_pose)
    for i in range(1, step):
        thetha = 6.28/step*i
        start_pose.position.y = center_a + math.cos(thetha)*radius
        start_pose.position.z = center_b + math.sin(thetha)*radius

        # y = center_a + math.cos(thetha)*radius
        # z = center_b + math.sin(thetha)*radius
        # y_list.append(y)
        # z_list.append(z)

        waypoints.append(start_pose)

    # plt.plot(y_list, z_list)
    # plt.show()

    ##----------------------------------------------
    fraction = 0.0  # 路径规划覆盖率
    maxtries = 100  # 最大尝试规划次数
    attempts = 0  # 已经尝试规划次数

    arm.set_start_state_to_current_state()

    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
    while fraction < 0.95 and attempts < maxtries:
        (plan, fraction) = arm.compute_cartesian_path(
            waypoints,  # waypoint poses，路点列表
            0.01,  # eef_step，终端步进值
            # TODO
            0.0,  # jump_threshold，跳跃阈值
            True)  # avoid_collisions，避障规划

        attempts += 1  # 尝试次数累加

        # 打印运动规划进程
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

    # 如果路径规划成功（覆盖率95%）,则开始控制机械臂运动
    if fraction > 0.95:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        arm.execute(plan)
        rospy.loginfo("Path execution complete.")
    else:
        # 如果路径规划失败，则打印失败信息
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(
            maxtries) + " attempts.")

    rospy.sleep(1)

    ######################################################
    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()
