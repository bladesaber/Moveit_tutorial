#!/usr/bin/env python2

import rospy
from demo.msg import demo_msg

def shutdown():
    rospy.loginfo("Stopping the robot...")

def callback(msg):
    print 'name:',msg.name
    print 'age:', msg.age, type(msg.age)
    print 'score:', msg.score, type(msg.score)

if __name__ == '__main__':
    rospy.on_shutdown(shutdown)

    rospy.init_node('sub_msg')

    sub = rospy.Subscriber('simple_topic', demo_msg, callback=callback )
    rospy.spin()
