#!/usr/bin/env python2

import rospy
import actionlib
from learning_communication.msg import DoDishesAction, DoDishesGoal, DoDishesFeedback

def execute(goal):
    rospy.loginfo("Dishwasher %d is working."%goal.dishwasher_id)

    for i in range(10):
        feedback = DoDishesFeedback()
        feedback.percent_complete = i*10
        server.publish_feedback(feedback)
        rate.sleep()

    rospy.loginfo("Dishwasher %d finish working."%goal.dishwasher_id)
    server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node("do_dishes_server")

    server = actionlib.SimpleActionServer("do_dishes", DoDishesAction, execute_cb=execute, auto_start=False)

    rate = rospy.Rate(1)

    server.start()

    rospy.spin()
