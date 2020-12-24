#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

def chatterCallback(msg):
    rospy.loginfo("I heard: [%s]", msg.data)

if __name__ == '__main__':
    rospy.init_node("listener")

    sub = rospy.Subscriber("chatter", String, callback=chatterCallback, queue_size=1000)
    rospy.spin()

