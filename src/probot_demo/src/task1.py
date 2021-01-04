#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped
from copy import deepcopy

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_fk_demo", anonymous=True)

    arm = moveit_commander.MoveGroupCommander('manipulator')
    arm.set_goal_joint_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    end_effector_link = arm.get_end_effector_link()
    arm.set_pose_reference_frame('base_link')
    arm.allow_replanning(True)

    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    # from A -> B
    joint_positions = [0.2, 0.2, 0.2, 0.3, -0.2, -0.2]
    arm.set_joint_value_target(joint_positions)

    arm.go()
    rospy.sleep(1)

    # from B -> C
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = 0.2
    target_pose.pose.position.y = 0.05
    target_pose.pose.position.z = 0.2
    target_pose.pose.orientation.x = 0.2
    target_pose.pose.orientation.y = 0.2
    target_pose.pose.orientation.z = 0.3
    target_pose.pose.orientation.w = 1.0

    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose, end_effector_link)

    traj = arm.plan()
    arm.execute(traj)

    rospy.sleep(1)

    # from C -> D -> E
    waypoints = []
    cur_pose = arm.get_current_pose(end_effector_link).pose
    waypoints.append(cur_pose)

    wpose = deepcopy(cur_pose)
    wpose.position.z -= 0.2
    wpose.position.x += 0.15
    waypoints.append(wpose)

    wpose = deepcopy(wpose)
    wpose.position.x -= 0.15
    wpose.position.y -= 0.1
    waypoints.append(wpose)

    fraction = 0.0  # 路径规划覆盖率
    maxtries = 100  # 最大尝试规划次数
    attempts = 0  # 已经尝试规划次数

    arm.set_start_state_to_current_state()

    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
    while fraction < 0.98 and attempts < maxtries:
        (plan, fraction) = arm.compute_cartesian_path(
            waypoints,  # waypoint poses，路点列表
            0.01,  # eef_step，终端步进值
            0.0,  # jump_threshold，跳跃阈值
            True)  # avoid_collisions，避障规划

        attempts += 1  # 尝试次数累加

        # 打印运动规划进程
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

    # 如果路径规划成功（覆盖率95%）,则开始控制机械臂运动
    if fraction > 0.98:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        arm.execute(plan)
        rospy.loginfo("Path execution complete.")
    else:
        # 如果路径规划失败，则打印失败信息
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(
            maxtries) + " attempts.")

    rospy.sleep(1)

    # # from E -> A
    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()

