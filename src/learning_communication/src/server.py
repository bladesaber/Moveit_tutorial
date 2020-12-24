#!/usr/bin/env python2

import rospy
from learning_communication.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

def add(requests):
    rospy.loginfo("request: x=%d, y=%d"%(requests.a, requests.b))
    rospy.loginfo("sending back response: [%d]"%(requests.a+requests.b))
    return AddTwoIntsResponse(requests.a + requests.b)

if __name__ == '__main__':
    rospy.init_node('add_two_ints_server')

    service = rospy.Service('add_two_ints', AddTwoInts, add)

    rospy.spin()
