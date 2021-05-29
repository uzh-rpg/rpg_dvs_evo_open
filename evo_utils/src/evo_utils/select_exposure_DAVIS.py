#!/usr/bin/env python

import roslib
import scipy.stats
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
import dynamic_reconfigure.client
import rospy
PACKAGE = 'davis_ros_driver'
roslib.load_manifest(PACKAGE)


"""
Selects automatically the exposure for the davis sensor communicating 
with the dynamic reconfigure service "/davis_ros_driver"

@see computeEntropy, computeSumGradients, the score functions that can be 
used to evaluate the exposure quality. Set the desired one in 
ExposureSelector.__init__, as self.computeScore = [desired score function]
"""


class ExposureSelector:

    def __init__(self):
        self.bridge = CvBridge()
        self.client = dynamic_reconfigure.client.Client(
            "/davis_ros_driver", timeout=10, config_callback=self.exposureCallback)
        rospy.Subscriber("image", Image, self.imageCallback)

        self.rate = 15
        self.computeScore = self.computeEntropy
        # self.computeScore = self.computeSumGradients # very sensitive to noise

        self.reset()

        self.selectOptimalExposureCallback()

    def reset(self):
        """
        Reset scores and current exposure
        """
        self.current_exposure = None
        self.scores = {}

    def exposureCallback(self, config):
        """
        Callback called when the dynamic reconfigure service changes the exposure.
        Can be used to check the correctness of the update.

        @config new configuration
        """
        rospy.loginfo('Set exposure: {}'.format(config['exposure']))

    def runPass(self, exposure_range, rate):
        """
        For each possible exposure in exposure_range, evaluates the corresponding score 
        by waiting a new image after the exposure is selected. To do this, it works at a
        frequency of rate Hz.

        @param exposure_range possible values among which the optimal exposure has to be found
        @param rate evaluation rate, used to wait imageCallback

        @return the optimal exposure in the range given

        @see imageCallback
        """
        r = rospy.Rate(rate)
        for i, exposure in enumerate(exposure_range):
            if rospy.is_shutdown():
                break

            self.current_exposure = exposure
            self.client.update_configuration(
                {"exposure": self.current_exposure})
            r.sleep()

        finished = (i >= (len(exposure_range)-1))
        if finished:
            optimal_exposure = max(self.scores, key=self.scores.get)
            self.reset()
            return optimal_exposure  # an optimal exposure has been found
        else:
            return -1

    def selectOptimalExposureCallback(self):
        """
        Performs the automatic exposure selection algorithm

        Two steps are performed. First, a first optimal exposure is found 
        with a coarse search in the range 100 <-> 27100. 
        Then, the exposure is refined with a finer search in the range 
        0.9 * first_solution <-> 1.1 first_solution.

        @see runPass
        """
        rospy.loginfo('Starting automatic exposure selection algorithm')

        # Start searching for the optimal exposure
        best_exp_pass1 = self.runPass(range(100, 27100, 3000), rate=self.rate)
        # first pass, find coarse exposure
        if best_exp_pass1 > 0:
            rospy.loginfo('First pass: success!')
            rospy.loginfo(
                'Best exposure in first pass: {}'.format(best_exp_pass1))

            # second pass, refine the exposure
            exp_range = [int(exp) for exp in np.linspace(
                0.9*best_exp_pass1, 1.1*best_exp_pass1, 10)]
            best_exp_pass2 = self.runPass(exp_range, rate=self.rate)

            if best_exp_pass2 > 0:
                rospy.loginfo('Second pass: success!')
                rospy.loginfo(
                    'Best exposure in second pass: {}'.format(best_exp_pass2))
                optimal_exposure = best_exp_pass2
            else:
                rospy.logerr(
                    'Second pass: failure\nSetting the best exposure found in first pass')
                optimal_exposure = best_exp_pass1

            self.client.update_configuration({"exposure": optimal_exposure})
            self.reset()

        else:
            rospy.logerr(
                'First pass: failure\nCould not auto-set the exposure')

    def computeEntropy(self, img):
        """
        Computes image entropy as the entropy of the histogram of intensities

        @param img the image of which the entropy has to be calculated

        @return image entropy
        """
        hist, bins = np.histogram(img.ravel(), bins=256, density=True)
        return scipy.stats.entropy(hist)

    def (self, img):
        """
        Computes Ix.^2 + Iy.^2, the sum of the squared (pixel-wise) gradients

        @param img image to calculate the gradients on

        @return sum of squared (pixel-wise) gradients
        """
        gradx, grady = cv2.Sobel(img, cv2.CV_8U, 1, 0, ksize=5), cv2.Sobel(
            img, cv2.CV_8U, 0, 1, ksize=5)
        #cv2.imshow("gradx", gradx)
        #cv2.imshow("grady", grady)
        # cv2.waitKey(10)
        return np.sum(np.power(gradx, 2) + np.power(grady, 2))

    def imageCallback(self, msg):
        """
        After a new image is received, if it has been received with an exposure
        for which no score is available, compute the score.

        @param msg message containing the image received

        @see computeEntropy, computeSumGradients which are the available score functions
        """
        if self.current_exposure and (not self.current_exposure in self.scores):
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

            self.scores[self.current_exposure] = self.computeScore(self.img)
            #rospy.loginfo('Score: {}'.format(self.scores[self.current_exposure]))


if __name__ == "__main__":
    rospy.init_node("exposure_selector_DAVIS")

    exposure_selector = ExposureSelector()
    rospy.spin()
