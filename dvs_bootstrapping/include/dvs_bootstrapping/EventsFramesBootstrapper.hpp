#pragma once

#ifndef KLT_BOOTSTRAPPER_HPP
#define KLT_BOOTSTRAPPER_HPP

#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <deque>
#include <opencv2/core.hpp>
#include <vector>

#include "dvs_bootstrapping/Bootstrapper.hpp"
#include "motion_correction.hpp"

namespace dvs_bootstrapping {

/**
 * Collects events and publishes motion undistorted events frames
 */
class EventsFramesBootstrapper : public Bootstrapper {
   public:
    EventsFramesBootstrapper(ros::NodeHandle &nh, ros::NodeHandle &nhp);
    virtual ~EventsFramesBootstrapper();

   protected:
    void postCameraLoaded() override;
    size_t width_;   ///< image size: width
    size_t height_;  ///< image size: height
    cv::Size sensor_size_;
    std::vector<cv::Point2f> rectified_points_;
    bool keep_frames_ = false;
    std::mutex img_mutex_;
    std::deque<cv::Mat> frames_;  ///< images obtained aggregating events
    int rate_hz_;
    ros::Time ts_;  ///< last timestamped image (last received event)

   private:
    tf::Transformer tf_;
    tf::TransformBroadcaster tf_pub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_event_img_, pub_optical_flow_;

    int frame_size_;        ///< number of events to accumulate in each frame
                            ///< ROS param: "frame_size"
    int local_frame_size_;  ///< how many events have to be used for the motion
                            ///< compensation
                            ///< ROS param: "local_frame_size"
    int newest_processed_event_;  ///< keeps track of the last processed event
                                  ///< ROS param: "n_min_events_frames"
    bool enable_visuals_;         ///< whether to publish visuals
                                  ///< ROS param: "enable_visualizations"
    cv::Mat undistort_mapx_;      ///< distorted to undistorted pixels (x)
    cv::Mat undistort_mapy_;      ///< distorted to undistorted pixels (y)

    motion_correction::WarpUpdateParams
        unwarp_params_;  /// motion undistortion params

    /**
     * Thread that if not idle calls integrateEvents
     *
     * @see integrateEvents
     */
    void integratingThread();

    /**
     * Performs motion compensation and publishes/stores the undistorted events
     * frame
     *
     *     ROS params:
     *          frame_size) number of events to aggregate in an events frame
     *          local_frame_size) size of the two batches of events used to
     *              compute the warp parameters unwarp_estimate_n_it) maximum
     *              number of iterations to
     *           estimate homography unwarp_estimate_eps) homography estimation
     *              tolerance
     *          unwarp_estimate_pyramid_lvls) pyramid levels used to estimate
     *              homography
     *          min_step_size) minimum number of new events before next events
     *              frame
     *          events_scale_factor)
     *              pixel intensity = sat(#events / events_scale_factor)
     *          adaptive_thresholding) whether to perform adaptive thresholding
     *              if adaptive_thresholding:
     *                  activation_threshold_min) minimum pixel intensity
     *                  activation_threshold_patch_size) filter patch size
     *          median_filtering) whether to perform median filtering
     *              if median_filtering:
     *                  median_filter_size) filter size
     *
     * If param "enable_visualizations" is true, it publishes the undistorted
     * events and the estimated optical flow.
     *
     * Moreover:
     *
     *      max_events_frames_saved_to_file) # of events frames stored on disk
     *      events_frames_filename_format) filename of the events frames stored
     *
     * @return Whether the motion undistortion was performed (i.e., there were
     * enough new events)
     */
    bool integrateEvents();

    /**
     * Clears events that are not useful anymore
     */
    void clearEventQueue();

    /**
     * Publishes image img at timestamp ts
     *
     *     ROS params:
     *         motion_corrected_topic) topic of the ros message
     *
     * @param img the image to publish
     * @param ts timestamp of the ros message
     */
    void publishEventImage(const cv::Mat &img, const ros::Time &ts);

    /**
     * Publishes optical flow field flow_field
     *
     *     ROS params:
     *         optical_flow_topic) topic of the ros message
     *
     * @param flow_field the optical flow field to publish
     */
    void publishOpticalFlowVectors(const cv::Mat &flow_field);
};

}  // namespace dvs_bootstrapping

#endif