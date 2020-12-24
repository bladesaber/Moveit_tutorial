#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

def shutdown():
    rospy.loginfo("Stopping the robot...")

def callback(msg):
    print msg.data

if __name__ == '__main__':
    rospy.on_shutdown(shutdown)

    rospy.init_node('sub_simple')

    sub = rospy.Subscriber('simple_topic', String, callback=callback )
    rospy.spin()
