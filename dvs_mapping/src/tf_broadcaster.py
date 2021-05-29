#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped


def handle_dvs_pose(msg):
    br = tf.TransformBroadcaster()
    t = msg.pose.position
    q = msg.pose.orientation
    br.sendTransform((t.x, t.y, t.z),
                     (q.x, q.y, q.z, q.w),
                     msg.header.stamp,
                     "dvs",
                     "world")


if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/dvs/pose',
                     PoseStamped,
                     handle_dvs_pose)

    rospy.spin()
