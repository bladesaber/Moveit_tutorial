#!/usr/bin/env python2

import rospy
from learning_communication.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

if __name__ == '__main__':
    rospy.init_node("add_two_ints_client")

    # assume
    a, b = 10, 20

    rospy.wait_for_service("add_two_ints")
    client = rospy.ServiceProxy("add_two_ints", AddTwoInts)
    srv = AddTwoIntsRequest()
    srv.a = a
    srv.b = b

    result = client(srv)
    rospy.loginfo('result:%d'%result.sum)
