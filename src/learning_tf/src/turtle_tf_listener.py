#!/usr/bin/env python2

import turtlesim
import rospy
import tf
import tf_conversions
from tf.broadcaster import TransformBroadcaster
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import tf2_ros
from turtlesim.srv import Spawn, SpawnRequest
import math

if __name__ == '__main__':
    rospy.init_node("my_tf_listener")

    turtle_name = rospy.get_param('/turtle', '/turtle2')

    turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, Twist, queue_size=1)

    rospy.wait_for_service("spawn")
    add_turtle = rospy.ServiceProxy("spawn", Spawn)

    srv = SpawnRequest()
    srv.x, srv.y, srv.theta, srv.name = 4, 2, 0, 'turtle2'
    add_turtle(srv)

    # tf 2.0
    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(buffer=tfBuffer)

    # tf 1.0
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # tf 2.0
            # trans = tfBuffer.lookup_transform(turtle_name, '/turtle1', rospy.Time(), timeout=rospy.Duration(3.0))
            # tf 1.0
            (trans, rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(rospy.Duration(1))
            continue

        msg = Twist()

        # tf 2.0
        # msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        # tf 1.0
        msg.angular.z = 4 * math.atan2(trans[1], trans[0])
        msg.linear.x = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)

        turtle_vel.publish(msg)

        rate.sleep()
