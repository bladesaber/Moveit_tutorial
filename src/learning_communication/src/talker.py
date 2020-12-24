#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node("talker")

    pub = rospy.Publisher('chatter', String, queue_size=1000)
    rate = rospy.Rate(10)

    count = 0
    while not rospy.is_shutdown():
        txt = "hello world %d"%count
        pub.publish(txt)
        rate.sleep()

        rospy.loginfo(txt)

        count += 1
