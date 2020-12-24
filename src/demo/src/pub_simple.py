#!/usr/bin/env python2

import rospy
from std_msgs.msg import String

def shutdown():
    rospy.loginfo("Stopping the robot...")

if __name__ == '__main__':
    rospy.on_shutdown(shutdown)

    rospy.init_node('pub_simple')

    pub = rospy.Publisher('simple_topic', String, queue_size=10)
    rate = rospy.Rate(2)

    test_param = rospy.get_param('~demo_param')
    print 'get demo param:',test_param

    while not rospy.is_shutdown():
        text = "hello world %s" % rospy.get_time()
        pub.publish(text)
        rate.sleep()
