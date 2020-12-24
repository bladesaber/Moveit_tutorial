#!/usr/bin/env python2

import rospy
import actionlib

from demo.msg import timerAction, timerGoal, timerFeedback

def parse_feedback(feedback):
    print('[Feedback] Time elapsed: %f' % (feedback.time_elapsed))
    print('[Feedback] Time remaining: %f' % (feedback.time_remaining))

if __name__ == '__main__':
    rospy.init_node('action_client')

    client = actionlib.SimpleActionClient('timer', timerAction)
    client.wait_for_server()

    # easy use
    # goal = timerGoal()
    # goal.time_to_wait = 5
    # client.send_goal(goal)
    #
    # client.wait_for_result()
    # print 'Get result:', client.get_result(), ' state:', client.get_state()

    # more use
    goal = timerGoal()
    goal.time_to_wait = 5
    client.send_goal(goal, feedback_cb=parse_feedback)
    client.wait_for_result()
    print 'Get result:', client.get_result(), ' state:', client.get_state()

    rospy.spin()