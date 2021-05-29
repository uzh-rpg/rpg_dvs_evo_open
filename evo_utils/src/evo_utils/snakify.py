#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


from tf2_msgs.msg import TFMessage

"""
Converts history of poses into a trajectory (snake shape)
param: "length" is the length of the history of poses to 
take into account when plotting the trajectory 
"""


class snakify:

    def __init__(self):
        self.length = rospy.get_param('~length', 500)
        self.frame_id = rospy.get_param('dvs_frame_id', 'dvs_evo')
        self.poses = []

        self.snake_pub = rospy.Publisher("snake", Path, queue_size=1)
        rospy.Subscriber("tf", TFMessage, self._TFCallback)

    def _TFCallback(self, msg):
        """
        Convert history of poses in trajectory ("snake")

        @param msg message containing the last transformation
        """
        if msg.transforms[0].child_frame_id != self.frame_id:
            return

        header = msg.transforms[0].header
        p = msg.transforms[0].transform.translation
        q = msg.transforms[0].transform.rotation

        P = PoseStamped()
        P.header = header
        P.pose.position = p
        P.pose.orientation = q

        self.poses.append(P)

        if self.length > 0 and len(self.poses) > self.length:
            del self.poses[:-1-self.length]

        path = Path()
        path.header = header
        path.poses = self.poses
        self.snake_pub.publish(path)


if __name__ == '__main__':
    rospy.init_node('snakify')
    node = snakify()
    rospy.spin()
