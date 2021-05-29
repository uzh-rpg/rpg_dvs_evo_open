#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
import numpy as np

from tf2_msgs.msg import TFMessage
from tf import TransformListener

"""
Publishes the transformation received to a camera representation for rviz visualization

topic: "dvs/camera_marker"
"""


class tfToCamera:

    def __init__(self):
        rospy.Subscriber("tf", TFMessage, self._TFCallback)
        self.frame_dvs = rospy.get_param('dvs_frame_id', 'dvs_evo')
        self.frame_gt = rospy.get_param(
            'dvs_groundtruth_frame_id', 'dvs_groundtruth')

        self.tf = TransformListener(True, rospy.Duration(1000.))
        self.marker_scale = rospy.get_param('~marker_scale', 0.2)

        self.pub_dvs = rospy.Publisher(
            'dvs/camera_marker', Marker, queue_size=0)
        self.pub_dvs_gt = rospy.Publisher(
            'dvs_gt/camera_marker', Marker, queue_size=0)

    def _TFCallback(self, msg):
        """
        For each transormation received, if the frame is either the 
        self.frame_dvs or the self.frame_gt, publishes a camera marker

        @param msg message containing the transformation 
        """
        transform = msg.transforms[0]
        frame = transform.child_frame_id
        if frame == self.frame_dvs:
            self._publishCameraMarker(self.pub_dvs, msg, color=(
                0.0, 0.0, 1.0), marker_scale=self.marker_scale)
        elif frame == self.frame_gt:
            self._publishCameraMarker(self.pub_dvs_gt, msg, color=(
                1.0, 0.0, 0.0), marker_scale=self.marker_scale)

    def _publishCameraMarker(self, pub, tf_msg, marker_scale=0.2, color=(1.0, 0.0, 0.0)):
        """
        Publishes a camera marker

        @param pub camera marker ros publisher
        @param tf_msg transformation message received
        @param marker_scale scale of the marker
        @param color scale of the marker
        """
        transform = tf_msg.transforms[0]
        frame = transform.child_frame_id
        stamp = transform.header.stamp

        sqrt2_2 = 0.5 * np.sqrt(2)

        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = ''
        marker.action = 0
        marker.id = 15

        # make rectangles as frame
        r_w = 1.0
        z_plane = (r_w / 2.0)*marker_scale
        marker.pose.position.x = 0
        marker.pose.position.y = (r_w / 4.0) * marker_scale
        marker.pose.position.z = z_plane

        marker.type = Marker.CUBE
        marker.scale.x = r_w*marker_scale
        marker.scale.y = 0.04*marker_scale
        marker.scale.z = 0.04*marker_scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.id -= 1
        pub.publish(marker)

        marker.pose.position.y = -(r_w / 4.0)*marker_scale
        marker.id -= 1
        pub.publish(marker)

        marker.scale.x = (r_w/2.0)*marker_scale
        marker.pose.position.x = (r_w / 2.0) * marker_scale
        marker.pose.position.y = 0
        marker.pose.orientation.w = sqrt2_2
        marker.pose.orientation.z = sqrt2_2
        marker.id -= 1
        pub.publish(marker)

        marker.pose.position.x = -(r_w / 2.0) * marker_scale
        marker.id -= 1
        pub.publish(marker)

        # make pyramid edges
        marker.scale.x = (3.0*r_w/4.0)*marker_scale
        marker.pose.position.z = 0.5*z_plane

        marker.pose.position.x = (r_w / 4.0) * marker_scale
        marker.pose.position.y = (r_w / 8.0) * marker_scale

        # 0.08198092, -0.34727674,  0.21462883,  0.9091823
        marker.pose.orientation.x = 0.08198092
        marker.pose.orientation.y = -0.34727674
        marker.pose.orientation.z = 0.21462883
        marker.pose.orientation.w = 0.9091823
        marker.id -= 1
        pub.publish(marker)

        marker.pose.position.x = -(r_w / 4.0) * marker_scale
        marker.pose.position.y = (r_w / 8.0) * marker_scale
        # -0.27395078, -0.22863284,  0.9091823 ,  0.21462883
        marker.pose.orientation.x = 0.08198092
        marker.pose.orientation.y = 0.34727674
        marker.pose.orientation.z = -0.21462883
        marker.pose.orientation.w = 0.9091823
        marker.id -= 1
        pub.publish(marker)

        marker.pose.position.x = -(r_w / 4.0) * marker_scale
        marker.pose.position.y = -(r_w / 8.0) * marker_scale
        # -0.08198092,  0.34727674,  0.21462883,  0.9091823
        marker.pose.orientation.x = -0.08198092
        marker.pose.orientation.y = 0.34727674
        marker.pose.orientation.z = 0.21462883
        marker.pose.orientation.w = 0.9091823
        marker.id -= 1
        pub.publish(marker)

        marker.pose.position.x = (r_w / 4.0) * marker_scale
        marker.pose.position.y = -(r_w / 8.0) * marker_scale
        # -0.08198092, -0.34727674, -0.21462883,  0.9091823
        marker.pose.orientation.x = -0.08198092
        marker.pose.orientation.y = -0.34727674
        marker.pose.orientation.z = -0.21462883
        marker.pose.orientation.w = 0.9091823
        marker.id -= 1
        pub.publish(marker)


if __name__ == '__main__':

    rospy.init_node('tf_to_camera_markers')

    tf_to_camera = tfToCamera()

    rospy.spin()
