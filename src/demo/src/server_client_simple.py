#!/usr/bin/env python2

import rospy
from demo.srv import demo_srv

def shutdown():
    rospy.loginfo("Stopping the robot...")

if __name__ == '__main__':
    rospy.on_shutdown(shutdown)

    rospy.init_node('server_client_simple')

    rospy.wait_for_service('word_count')
    word_counter = rospy.ServiceProxy('word_count', demo_srv)

    words = 'hello world h h h h h h  h'
    word_count = word_counter(words)

    print 'word count: ', word_count.count
