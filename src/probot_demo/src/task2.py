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

    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    arm.set_pose_reference_frame('base_link')
    arm.allow_replanning(True)

    scene = PlanningSceneInterface()
    rospy.sleep(1)

    arm.set_named_target('home')
    arm.go()

    #############################################
    scene.remove_attached_object(end_effector_link, 'tool')
    scene.remove_world_object('obstacle')
    scene.remove_world_object('target')

    obstacle_size = [0.2, 0.05, 0.01]
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

    scene.attach_box(end_effector_link, 'tool', p, tool_size)

    # ############################################
    # 将obstacle加入场景当中
    obstacle_pose = PoseStamped()
    obstacle_pose.header.frame_id = 'base_link'
    obstacle_pose.pose.position.x = 0.35
    obstacle_pose.pose.position.y = 0.0
    obstacle_pose.pose.position.z = 0.33
    obstacle_pose.pose.orientation.w = 1.0
    scene.add_box('obstacle', obstacle_pose, obstacle_size)

    rospy.sleep(2)

    ############################################
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = 0.2593
    target_pose.pose.position.y = 0.0636
    target_pose.pose.position.z = 0.1787
    target_pose.pose.orientation.x = 0.70692
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.70729

    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose, end_effector_link)

    traj = arm.plan()
    arm.execute(traj)
    rospy.sleep(1)

    # 控制机械臂回到初始化位置
    arm.set_named_target('home')
    arm.go()

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()
