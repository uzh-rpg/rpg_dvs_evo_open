#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from dvs_slam_msgs.msg import VoxelGrid
from rospy.numpy_msg import numpy_msg
import matplotlib.cm
import colormaps as cmaps

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

cv_bridge = CvBridge()
image_pub = rospy.Publisher("dsi/confidence",  Image, queue_size=10)
cmap = matplotlib.cm.get_cmap('coolwarm')


def reorder_axes(arr):
    """
    Shift right axes of arr, i.e., from (0,1,2) to (2,0,1)

    @param arr array of which the axes have to be shifted
    """
    arr = np.swapaxes(arr, 0, 2)
    arr = np.swapaxes(arr, 1, 2)
    return arr


def normalize(img):
    """
    Scales image values between 0 and 1 (min-max normalization)

    @param img the image to normalize
    """
    return cv2.normalize(img, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)


def publish_confidence(voxel_grid_msg):
    """
    Compute confidence for the received voxel grid and publishes it

    Topic: "voxel_grid"

    @param voxel_grid_msg message containing the voxel grid
    """
    msg = voxel_grid_msg.voxel_grid
    h = msg.layout.dim[0].size
    w = msg.layout.dim[1].size
    N = msg.layout.dim[2].size

    msg.data.setflags(write=True)
    arr = reorder_axes(msg.data.reshape((h, w, N)))  # axes: N h w

    img = np.zeros((h+N, w+N), dtype=np.float32)
    img[0:h, 0:w] = normalize(-np.amax(arr, axis=0))
    img[h:h+N, 0:w] = normalize(-np.amax(arr, axis=1))
    img[0:h, w:w+N] = normalize(-np.amax(arr, axis=2).T)
    img[h+1:h+N, w+1:w+N] = 255
    img = (255. * cmap(img)[:, :, :3]).astype(np.uint8)

    c = (127, 127, 127)
    cv2.line(img, (0, h), (img.shape[1], h), c)
    cv2.line(img, (w, 0), (w, img.shape[0]), c)

    image_pub.publish(cv_bridge.cv2_to_imgmsg(img, encoding="bgr8"))


if __name__ == '__main__':
    topic_name = 'voxel_grid'
    rospy.init_node('publish_confidence')
    rospy.Subscriber(topic_name,
                     numpy_msg(VoxelGrid),
                     publish_confidence)
    rospy.spin()
