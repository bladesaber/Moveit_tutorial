#!/usr/bin/env python2

import rospy
import actionlib
from learning_communication.msg import DoDishesAction, DoDishesGoal, DoDishesFeedback

def doneCb(state, result):
    print 'state:', state
    print 'result:', result
    rospy.loginfo("Yay! The dishes are now clean")

def activeCb():
    rospy.loginfo("Goal just went active")

def feedbackCb(feedback):
    rospy.loginfo(feedback.percent_complete)

if __name__ == '__main__':
    rospy.init_node("do_dishes_client")

    client = actionlib.SimpleActionClient("do_dishes", DoDishesAction)
    rospy.loginfo("Waiting for action server to start.")
    client.wait_for_server()

    goal = DoDishesGoal()
    goal.dishwasher_id = 1

    client.send_goal(goal, done_cb=doneCb, active_cb=activeCb, feedback_cb=feedbackCb)
    rospy.spin()

