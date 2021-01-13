#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_trajectory')

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

    joint_value = arm.get_current_joint_values()
    joint_value[0] = -1.5
    joint_value[1] = -0.5

    arm.set_joint_value_target(joint_value)
    traj = arm.plan()

    points = traj.joint_trajectory.points
    new_points = []
    new_points.append(points[0])

    i = 1
    while i*4<(len(points)-1):
        new_points.append(points[i*4])
        i += 1
    new_points.append(points[-1])

    traj.joint_trajectory.points = new_points
    arm.execute(traj)

    rospy.sleep(1)

    ######################################################
    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()
