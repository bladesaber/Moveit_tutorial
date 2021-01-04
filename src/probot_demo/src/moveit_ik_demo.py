#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import sys

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_ik_demo', anonymous=True)

    arm = moveit_commander.MoveGroupCommander('manipulator')
    end_effector_link = arm.get_end_effector_link()

    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)

    arm.set_pose_reference_frame('base_link')
    arm.allow_replanning(True)

    arm.set_named_target('home')
    arm.go()

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

    arm.stop()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
    rospy.spin()
