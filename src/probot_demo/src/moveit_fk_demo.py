#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_fk_demo", anonymous=True)

    arm = moveit_commander.MoveGroupCommander('manipulator')
    arm.set_goal_joint_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
    arm.set_joint_value_target(joint_positions)

    arm.go()
    rospy.sleep(1)

    arm.set_named_target('home')
    arm.go()
    rospy.sleep(1)

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()
