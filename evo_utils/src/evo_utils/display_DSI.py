# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from dvs_slam_msgs.msg import VoxelGrid
from rospy.numpy_msg import numpy_msg


from vispy import app, scene
from vispy.visuals.transforms import STTransform

import time

fov = 0.
scale_factor = 500.
bgcolor = 'gray'
cmap = 'coolwarm'


canvas = scene.SceneCanvas(show=True, bgcolor=bgcolor, title='DSI')
view = canvas.central_widget.add_view()
view.camera = scene.cameras.TurntableCamera(
    parent=view.scene, fov=fov, scale_factor=scale_factor)

vol = scene.visuals.Volume(np.zeros((1, 1, 1)), parent=view.scene, cmap=cmap)
vol.relative_step_size = 1.


cv2.namedWindow('DSI', cv2.WINDOW_NORMAL)


def reorder_axes(arr):
    """
    Shift right axes of arr, i.e., from (0,1,2) to (2,0,1)

    @param arr array of which the axes have to be shifted
    """
    arr = np.swapaxes(arr, 0, 2)
    arr = np.swapaxes(arr, 1, 2)
    return arr


def updateDSI(voxel_grid_msg):
    """
    Updates the DSI in rviz from the received voxel_grid_msg

    @param voxel_grid_msg message containing the new voxel grid
    """
    msg = voxel_grid_msg.voxel_grid
    h = msg.layout.dim[0].size
    w = msg.layout.dim[1].size
    N = msg.layout.dim[2].size

    msg.data.setflags(write=True)
    arr = reorder_axes(msg.data.reshape((h, w, N)))
    clim = (np.min(arr), np.max(arr))

    vol.set_data(arr, clim)
    vol.update()

    if updateDSI.set_transform:
        z_scale = (w+h)/3./N
        vol.transform = scene.STTransform(
            scale=(1., 1., z_scale), translate=(-w/2, -h/2, -N/2*z_scale))
        updateDSI.set_transform = False


updateDSI.set_transform = True


if __name__ == '__main__':

    topic_name = '/dvs_mapping/voxel_grid'

    rospy.init_node('display_DSI')

    rospy.Subscriber(topic_name,
                     numpy_msg(VoxelGrid),
                     updateDSI)
    app.run()
