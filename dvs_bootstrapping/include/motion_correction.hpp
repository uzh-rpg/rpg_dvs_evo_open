#pragma once

#ifndef MOTION_CORRECTION_HPP
#define MOTION_CORRECTION_HPP

#include <dvs_msgs/Event.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <vector>

namespace motion_correction {

using EventArray = std::vector<dvs_msgs::Event>;

/**
 * Custom floor function, reduces overhead avoiding overflows check (not
 * required in our use cases)
 *
 * @param x number to round to the floor
 *
 * @return rounded value
 */
static inline int int_floor(float x) {
    int i = (int)x;     /* truncate */
    return i - (i > x); /* convert trunc to floor */
}

/**
 * Resets matrix if already initialized, otherwise instantiates a zero matrix
 *
 * @param arr matrix to reset
 * @param size size of the matrix
 * @param type type of the matrix (default: CV_32F)
 */
static inline void resetMat(cv::Mat& arr, const cv::Size& size,
                            int type = CV_32F) {
    if (arr.cols == 0 || arr.rows == 0) {
        arr = cv::Mat::zeros(size, type);
    } else {
        arr.setTo(0);
    }
}

/**
 * Embeds all the parameters required for the estimation of the warp
 */
class WarpUpdateParams {
   public:
    int warp_mode;              ///< currently supported: cv::MOTION_HOMOGRAPHY
    int num_pyramid_levels;     ///< # pyramid levels used to estimate the warp
    cv::Size sensor_size;       ///< image size
    cv::TermCriteria criteria;  ///< termination criteria for the optimization
    WarpUpdateParams() {}

    WarpUpdateParams(int nIt, double eps, int mode, int lvls,
                     cv::Size sensor_size)
        : warp_mode(mode),
          num_pyramid_levels(lvls),
          criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, nIt, eps),
          sensor_size(sensor_size) {}
};

/**
 * Initializes warp matrix for the multi-level analysis
 *
 * @param warp warp matrix to process
 * @param params optimization parameters (# pyramid levels)
 */
void initWarp(cv::Mat& warp, const WarpUpdateParams& params);

/**
 * Estimates the warp the better aligns img0 and img1
 *
 * @param warp warp estimate
 * @param img0 first frame
 * @param img1 second frame
 * @param params optimization parameters
 */
void updateWarp(cv::Mat& warp, const cv::Mat& img0, const cv::Mat& img1,
                const WarpUpdateParams& params);

/**
 * Computes optical flow from warp
 *
 * @param warp warp from which to compute the optical flow
 * @param dt time to estimate the velocity field
 * @param sensor_size optical flow image size
 * @param rectified_points rectification table
 *
 * @return computed optical flow
 */
cv::Mat computeFlowFromWarp(const cv::Mat& warp, double dt,
                            cv::Size sensor_size,
                            std::vector<cv::Point2f> rectified_points);

/**
 * Draw undistorted events on image
 *
 * @param ev_first iterator to first event
 * @param ev_last last iterator (included)
 * @param out output events frame
 * @param rectified_points rectification table
 * @param use_polarity whether to account for polarity when drawing events
 */
void drawEventsUndistorted(EventArray::iterator ev_first,
                           EventArray::iterator ev_last, cv::Mat& out,
                           cv::Size sensor_size,
                           const std::vector<cv::Point2f>& rectified_points,
                           const bool use_polarity);

/**
 * Draw motion corrected events from optical flow
 *
 * @param ev_first iterator to first event
 * @param ev_last last iterator (included)
 * @param flow_field optical flow to use to undistort events
 * @param out output events frame
 * @param sensor_size size of the events frame
 * @param rectified_points rectification table
 * @param use_polarity whether to account for polarity when drawing events
 */
void drawEventsMotionCorrectedOpticalFlow(
    EventArray::iterator ev_first, EventArray::iterator ev_last,
    const cv::Mat& flow_field, cv::Mat& out, cv::Size sensor_size,
    const std::vector<cv::Point2f>& rectified_points, const bool use_polarity);

}  // namespace motion_correction

#endif