#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from vikit_py import transformations as transformations
import numpy as np

# Read pose from topic and broadcast it
# to /tf for a fixed amount of time


class Boostrapper:

    def __init__(self, tf_frame_name, tf_world_name, relative_to_first_pose=True):
        self.br = tf.TransformBroadcaster()
        self.tf_frame_name = tf_frame_name
        self.tf_world_name = tf_world_name
        self.got_first_pose = False

        # whether the pose will be given relative to the first pose received
        self.relative_to_first_pose = relative_to_first_pose

    def handle_dvs_pose(self, msg):

        if not self.got_first_pose:
            # print('Broadcasting pose to tf frame: %s%s'.format(
            #     self.tf_world_name, self.tf_frame_name))

            # Keep first pose T_w_dvs to use as the reference coordinate system
            t = msg.pose.position
            q = msg.pose.orientation
            self.T_w_init = transformations.matrix_from_quaternion(
                np.array([q.x, q.y, q.z, q.w]))
            self.T_w_init[:3, 3] = np.array([t.x, t.y, t.z])

            self.got_first_pose = True

        # Transform the DVS pose into the reference coordinate system
        # defined by the first pose
        t = msg.pose.position
        t = np.array([t.x, t.y, t.z])
        q = msg.pose.orientation
        q = np.array([q.x, q.y, q.z, q.w])
        T_w_c = transformations.matrix_from_quaternion(q)
        T_w_c[:3, 3] = t

        if self.relative_to_first_pose:
            # Transform the DVS pose into the reference coordinate system
            # defined by the first pose
            T_init_c = np.linalg.inv(self.T_w_init).dot(T_w_c)
            t = T_init_c[:3, 3]
            q = transformations.quaternion_from_matrix(T_init_c)
        else:
            t = T_w_c[:3, 3]
            q = transformations.quaternion_from_matrix(T_w_c)

        self.br.sendTransform((t[0], t[1], t[2]),
                              (q[0], q[1], q[2], q[3]),
                              msg.header.stamp,
                              self.tf_frame_name,
                              self.tf_world_name)


if __name__ == '__main__':

    rospy.init_node('pose_to_tf')
    source_topic_name = rospy.get_param('~source_topic_name', '/dvs/pose')
    tf_frame_name = rospy.get_param('dvs_bootstrap_frame_id', '/dvs_bootstrap')
    tf_world_name = rospy.get_param('world_frame_id', '/world')
    relative_to_first_pose = rospy.get_param('~relative_to_first_pose', True)

    bootstrapper = Boostrapper(
        tf_frame_name, tf_world_name, relative_to_first_pose)

    rospy.Subscriber(source_topic_name,
                     PoseStamped,
                     bootstrapper.handle_dvs_pose)

    rospy.spin()
