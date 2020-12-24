#!/usr/bin/env python2

import rospy
from demo.msg import demo_msg

def shutdown():
    rospy.loginfo("Stopping the robot...")

if __name__ == '__main__':
    rospy.on_shutdown(shutdown)

    rospy.init_node('pub_msg')

    pub = rospy.Publisher('simple_topic', demo_msg, queue_size=10)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = demo_msg()
        msg.name = 'catkin'
        msg.age = 10
        msg.score = 60

        pub.publish(msg)
        rate.sleep()
