#!/usr/bin/env python2

import rospy
from demo.srv import demo_srv, demo_srvRequest, demo_srvResponse

def shutdown():
    rospy.loginfo("Stopping the robot...")

def parse(requests):
    print 'server receive: ',requests.words
    return demo_srvResponse(len(requests.words.split()))

if __name__ == '__main__':
    rospy.on_shutdown(shutdown)

    rospy.init_node('server_simple')

    service = rospy.Service('word_count', demo_srv, parse)

    rospy.spin()