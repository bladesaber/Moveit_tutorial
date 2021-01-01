#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_cartesian_demo', anonymous=True)

    arm = moveit_commander.MoveGroupCommander('manipulator')
    end_effector_link = arm.get_end_effector_link()

    arm.set_pose_reference_frame('base_link')
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)
    arm.allow_replanning(True)

    arm.set_named_target('home')
    arm.go()

    waypoints = []
    cur_pose = arm.get_current_pose(end_effector_link)
    waypoints.append(cur_pose)

    cur_pose.pose.position.z -= 0.2
    waypoints.append(cur_pose)

    cur_pose.pose.position.x += 0.1
    waypoints.append(cur_pose)

    cur_pose.pose.position.y += 0.1
    waypoints.append(cur_pose)

    ###----------------------------------------------
    fraction = 0.0  # 路径规划覆盖率
    maxtries = 100  # 最大尝试规划次数
    attempts = 0  # 已经尝试规划次数

    # 设置机器臂当前的状态作为运动初始状态
    arm.set_start_state_to_current_state()

    while fraction < 0.95 and attempts < maxtries:
        (plan, fraction) = arm.compute_cartesian_path(
            waypoints,  # waypoint poses，路点列表
            0.01,  # eef_step，终端步进值
            0.01,  # jump_threshold，跳跃阈值
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

    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()
