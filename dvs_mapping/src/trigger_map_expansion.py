#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
from rospy.numpy_msg import numpy_msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import String
from tf import TransformListener
from tf.transformations import quaternion_matrix
import cv2
import time
import yaml

# import datetime

# States
DISABLED = 'DISABLED'
WAIT_FOR_MAP = 'WAIT_FOR_MAP'
CHECKING = 'CHECKING'

"""
Evaluates the quality of the map and eventually trigger an update request to the mapper

topics:

Publishes "remote_key" -> "update" when an update is necessary

Listens  "remote_key" -> "enable_map_expansion" and "disable_map_expansion"
to enable/disable this service

    ROS params:
        ~rate) rate at which the node checks whether an expansion is needed
        ~visibility_threshold) visibility of the map below which the update is triggered
        ~baseline_threshold) baseline / mean depth ration above which the update is triggered
        ~coverage_threshold) minimum amount of pixels covered by reprojected map threshold
        dvs_frame_id) frame id of the camera
        world_frame_id) world frame used
        calib_file) calibration file for the camera
        camera_info) topic where camera info are published
        number_of_initial_maps_to_skip) starts checking updates conditions after this # of maps
"""


class TriggerMapExpansion:
    _state = WAIT_FOR_MAP
    _map = None
    _t_map = None
    _got_camera_info = False

    def __init__(self):
        self._rate = rospy.get_param('~rate', 3)
        self._visibility_th = rospy.get_param('~visibility_threshold', .75)
        self._coverage_th = rospy.get_param('~coverage_threshold', .3)
        self._baseline_th = rospy.get_param('~baseline_threshold', .1)
        self._dvs_frame_id = rospy.get_param('dvs_frame_id', 'dvs_evo')
        self._world_frame_id = rospy.get_param('world_frame_id', 'world')
        self._map_to_skip = rospy.get_param(
            'number_of_initial_maps_to_skip', 0)

        with open(rospy.get_param('calib_file', ''), 'r') as stream:
            try:
                cam_info = yaml.load(stream, yaml.SafeLoader)
                self._w = cam_info['image_width']
                self._h = cam_info['image_height']
                self._K = np.array(
                    cam_info['camera_matrix']['data']).reshape((3, 3))
            except yaml.YAMLError as exc:
                print(exc)

        self._remote = rospy.Publisher('remote_key', String, queue_size=1)

        self._tf = TransformListener(True)
        rospy.Subscriber("pointcloud", PointCloud2, self._MapCallback)
        rospy.Subscriber("remote_key", String, self._RemoteKeyCallback)
        rospy.Subscriber("camera_info", CameraInfo, self._CameraInfoCallback)
        rospy.Timer(rospy.Duration(1./self._rate), self._CheckNewMapNeeded)

        # cv2.namedWindow('mask', cv2.WINDOW_NORMAL)

    def _CameraInfoCallback(self, msg):
        """
        Update camera calibration once from topic "camera_info"
        """
        if self._got_camera_info:
            return

        self._w = msg.width
        self._h = msg.height
        self._K = np.array(msg.K).reshape((3, 3))

        self._got_camera_info = True

    def _RemoteKeyCallback(self, msg):
        """
        Listens to enable and disable map expansion commands
        topic: remote_key

        "disable_map_expansion" -> does not check whether a map update is needed
        "enable_map_expansion" -> checks whether a map update is needed

        @param msg remote key message
        """
        m = msg.data

        if (m == 'disable_map_expansion'):
            self._state = DISABLED
        if (m == 'enable_map_expansion'):
            self._state = CHECKING

    def _MapCallback(self, msg):
        """
        Store last published map

        @param msg message containing the last published map
        """
        if self._state == DISABLED:
            return

        map = []
        for p in pc2.read_points(msg):
            map.append([p[0], p[1], p[2], 1.])

        self._map = np.array(map).T
        rospy.loginfo('Received map: {} points'.format(len(map)))

        try:
            now = rospy.Time(0)
            self._tf.waitForTransform(
                self._dvs_frame_id, self._world_frame_id, now, rospy.Duration(1.))
            (self._t_map, q) = self._tf.lookupTransform(
                self._dvs_frame_id, self._world_frame_id, now)
            self._state = CHECKING
        except:
            self._t_map = None

    def _CheckNewMapNeeded(self, event):
        """
        Regularly check whether a map update is needed if self._state == CHECKING
        and a map has already been received

        @param rospy.Timer event

        @see _MapVisibility, _BaselineOverDepth
        """

        if self._state != CHECKING:
            return

        if (self._t_map and self._map.size == 0):
            return

        try:
            now = rospy.Time(0)
            self._tf.waitForTransform(
                self._dvs_frame_id, self._world_frame_id, now, rospy.Duration(1.))
        except:
            return

        (t, q) = self._tf.lookupTransform(
            self._dvs_frame_id, self._world_frame_id, now)

        T = quaternion_matrix(q)
        T[:3, 3] = t

        # Project map into camera frame
        pts = T.dot(self._map)[:3, ]

        # t1 = datetime.datetime.now()
        coverage, visibility = self._MapVisibility(pts)
        # t2 = datetime.datetime.now()
        BoD = self._BaselineOverDepth(
            pts, np.subtract(t, self._t_map)) if self._t_map else 0.
        # t3 = datetime.datetime.now()

        # t21 = t2 - t1
        # t32 = t3 - t2
        # print(
        #     'Map visibility: {}% -- {}ms, baseline/depth: {}% -- {}ms'.format(visibility,
        #       t21.total_seconds()*1000, BoD, t32.total_seconds()*1000))

        if coverage < self._coverage_th or visibility < self._visibility_th or BoD > self._baseline_th:
            rospy.loginfo(
                'Sending update, coverage: {} %, map visibility: {} %, baseline/depth: {}'.format(coverage * 100, visibility * 100, BoD))
            self._state = WAIT_FOR_MAP
            self._remote.publish('update')

    def _MapVisibility(self, pts):
        """
        Computes the map visibility as the number of points that are projected
        onto the new frame over the total number of points.

        @param pts map points

        @return the map visibility
        """

        pts = pts[:, pts[2, ] > 0]
        pts = self._K.dot(pts/pts[2, ]).T

        N = len(pts)

        if N == 0:
            return 0.

        mask = np.zeros((self._h, self._w), np.uint8)
        cnt = 0.
        for i in xrange(N):
            x, y = pts[i][:2]
            cv2.circle(mask, (int(x), int(y)), 7, 255, -1)
            if x >= 0 and y >= 0 and x < self._w and y < self._h:
                cnt += 1.

        # cv2.imshow('mask', mask)
        # cv2.waitKey(3)

        return float(cv2.countNonZero(mask))/(self._w*self._h), float(cnt) / N

    def _BaselineOverDepth(self, pts, t):
        """
        Computes the heuristic baseline / average depth

        @param pts map point cloud (3d points: 3xN)
        @patam t translation vector between the two poses
        """
        avg_depth = np.average(np.linalg.norm(pts, axis=0))
        baseline = np.linalg.norm(t)

        return baseline / avg_depth


if __name__ == '__main__':
    rospy.init_node('trigger_map_expansion')
    node = TriggerMapExpansion()
    rospy.spin()
