#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import sys

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_random_demo', anonymous=True)

    arm = moveit_commander.MoveGroupCommander('manipulator')

    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    arm.set_named_target('home')
    arm.go()

    arm.set_random_target()

    arm.go()
    rospy.sleep(1)

    arm.set_named_target('home')
    arm.go()

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()
