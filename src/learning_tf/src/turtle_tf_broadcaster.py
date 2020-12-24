#!/usr/bin/env python2

import turtlesim
import rospy
import tf
import tf_conversions
from tf.broadcaster import TransformBroadcaster
from turtlesim.msg import Pose
import geometry_msgs.msg
import tf2_ros

def poseCallback(msg):
    # tf 2.0
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtle_name
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)

    # tf 1.0
    # br.sendTransform(
    #     translation=(msg.x, msg.y, 0),
    #     rotation=tf.transformations.quaternion_from_euler(0, 0, msg.theta),
    #     time=rospy.Time.now(),
    #     child=turtle_name,
    #     parent="world"
    # )

if __name__ == '__main__':
    rospy.init_node('"my_tf_broadcaster"')

    # tf 2.0
    br = tf2_ros.TransformBroadcaster()
    # tf 1.0
    # br = TransformBroadcaster()

    turtle_name = rospy.get_param('~turtle_name', '/turtle')
    print 'sub %s'%turtle_name
    rospy.Subscriber(turtle_name+"/pose", Pose, poseCallback)

    rospy.spin()