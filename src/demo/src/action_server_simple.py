#!/usr/bin/env python2

import rospy
import actionlib
from demo.msg import timerAction, timerGoal, timerResult, timerFeedback
import time

def do_timer(goal):
    # easy use
    # time.sleep(goal.time_to_wait)
    # result = timerResult()
    # result.time_elapsed = time.time()
    # server.set_succeeded(result)

    # more use
    start_time = time.time()
    finish_time = start_time + goal.time_to_wait

    current = time.time()
    while current < finish_time:

        if server.is_preempt_requested():
            result = timerResult()
            result.time_elapsed = current-start_time
            server.set_preempted(result)

        feed_back = timerFeedback()
        feed_back.time_elapsed = current - start_time
        feed_back.time_remaining = finish_time - current
        current = time.time()

        server.publish_feedback(feed_back)
        time.sleep(1)

    result = timerResult()
    result.time_elapsed = current - start_time
    server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('action_server')

    server = actionlib.SimpleActionServer('timer', timerAction, do_timer, False)
    server.start()

    rospy.spin()

