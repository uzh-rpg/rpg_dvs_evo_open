#ifndef TRACKER_H
#define TRACKER_H

#include <dvs_msgs/EventArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <mutex>

#include "dvs_tracking/lk_se3.hpp"

#define TRACKER_EVENT_HISTORY_SIZE \
    500000  // size of the history of events (already processed) to keep

// #define TRACKING_LOST_RESET \
//     2000  // if defined, reset tracking when keypoints_.size() < 5000

#define TRACKER_DEBUG_REFERENCE_IMAGE  // define this to start overlap thread
// #define TRACKING_PERF  // define this to evaluate also the performances

/**
 * Tracking node
 */
class Tracker : public LKSE3 {
   public:
    /**
     *      ROS params passed to LKSE3:
     *          batch_size) LKSE3::batch_size_
     *              batch-gradient descent batch size
     *          max_iterations) LKSE3::max_iterations_
     *              maximum number of iterations in the optimization
     *          map_blur) LKSE3::map_blur_
     *              sigma of the gaussian filter applied to the reprojected map
     *          pyramid_levels) LKSE3::pyramid_levels_
     *              number of pyramid levels used in the KLT process
     *          weight_scale_translation) LKSE3::weight_scale_translation_
     *              weight used in the translation error
     *          weight_scale_rotation) LKSE3::weight_scale_rotation_
     *              weight used in the rotation error
     *
     * @param nh ROS node handle
     * @param nh_private ROS private node handle
     */
    Tracker(ros::NodeHandle& nh, ros::NodeHandle nh_private);

   private:
    ros::NodeHandle nh_, nhp_;
    image_transport::ImageTransport it_;
    tf::Transformer tf_;
    tf::TransformBroadcaster tf_pub_;

    ros::Subscriber event_sub_;        ///< @see eventCallback
                                       ///< topic: "events"
    ros::Subscriber map_sub_;          ///< @see mapCallback
                                       ///< topic: "pointcloud"
    ros::Subscriber remote_sub_;       ///< @see remoteCallback
                                       ///< topic: "remote_key"
    ros::Subscriber tf_sub_;           ///< @see tfCallback
                                       ///< topic: "tf"
    ros::Subscriber camera_info_sub_;  ///< @see cameraInfoCallback
                                       ///< topic: "camera_info"

    ros::Publisher poses_pub_;  ///< @see publishPose
                                ///< topic: "evo/pose"

    std::string frame_id_;
    std::string world_frame_id_;

    bool idle_;  ///< whether the tracking is idle or active

    EventQueue events_;  ///< already processed and to process events

    size_t cur_ev_, kf_ev_, noise_rate_, frame_size_, step_size_, event_rate_;

    std::vector<tf::StampedTransform> poses_;           // estimated poses
    std::vector<tf::StampedTransform> poses_filtered_;  // median filtered poses

    std::mutex data_mutex_;  ///< mutex to own when accessing data

    bool auto_trigger_;

    /**
     * Stores incoming events and removes older ones
     *
     *     ROS params:
     *          discard_events_when_idle) whether not to collect events when
     * idle
     *
     * @param msg Message containing incoming events
     */
    void eventCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    /**
     * Updates map_ with the new published point cloud
     *
     * @param msg Message containing the point cloud representing the new map
     */
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    /**
     * On remote command, initializes or resets
     *
     *      Commands:
     *          switch) initialize
     *          reset) reset
     *          bootstrap) activate automatic startup procedure
     *
     * @see initialize, reset
     */
    void remoteCallback(const std_msgs::String::ConstPtr& msg);
    /**
     * Adds published transforms to the tf_ data structure
     */
    void tfCallback(const tf::tfMessagePtr& msgs);

    /**
     * Update camera info on callback
     *
     *
     * topic: "camera_info"
     *
     * @param msg Camera info msg
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    /**
     * Initializes the transformations of LKSE3 and update the map
     *
     *     ROS params:
     *         dvs_bootstrap_frame_id) get first pose from last published in
     *              this frame
     *
     * @param ts Timestamp at which initialize the pose (last reliable pose)
     *
     * @see updateMap
     */
    void initialize(const ros::Time& ts);

    /**
     * Sets the tracker state to idle_ = true and clears variables
     */
    void reset();

    /**
     * If active, regularly estimates trajectory based on currently stored
     * events
     *
     *     ROS params:
     *         max_event_rate) events processed are randomly sampled so that
     *              the rate is below this value
     *         events_per_kf) events required for a new keyframe
     *         frame_size) window of events considered
     *         step_size) minimum number of new events to wait before a pose
     *              update
     *         noise_rate) if events rate is below this value, the frame is
     *              skipped
     *
     *
     * @see estimateTrajectory
     */
    void estimateTrajectory();

    /**
     * Set current frame as reference frame and project the map
     *
     *     ROS params:
     *          min_map_size) minimum number of map points to proceed with the
     *              update
     *          min_n_keypoints) minimum number of extracted keypoints
     *                  (LKSE3::keypoints_) required for a reliable tracking
     *
     * @see LKSE3::projectMap
     */
    void updateMap();

    /**
     * Get median filtered poses
     *
     *     ROS params:
     *         pose_mean_filter_size) median filter size for the filtered
     *              trajectory
     *
     * @param pose [output] median filtered pose
     *
     * @return true if the median filtering has been performed and the pose
     * assigned, false otherwise
     */
    bool getFilteredPose(tf::StampedTransform& pose);

    /**
     * If active, regularly estimate trajectory based on currently stored events
     *
     * @see estimateTrajectory
     */
    void trackingThread();
    /**
     * Separates thread taking the event image (built by drawing the events
     * received onto the image) and overlaying the points in the local map
     *
     *     ROS params:
     *         event_map_overlap_rate) publishing rate of the thread
     *         max_depth) maximum expected depth in the scene
     *         min_depth) minimum expected depth in the scene
     *
     * topic: "event_map_overlap", where the image is published
     */
    void publishMapOverlapThread();
    /**
     * Publishes new median filtered pose, after building the last pose
     *
     * @see publishPose, getFilteredPose
     */
    void publishTF();

    /**
     * Publishes last pose in poses_
     *
     *     ROS params:
     *         world_frame_id) world frame id
     *         dvs_frame_id) frame id of the published pose
     */
    void publishPose();

    /**
     * Removes events to keep an history (already processed) of events at most
     * long TRACKER_EVENT_HISTORY_SIZE.
     */
    void clearEventQueue();

    /**
     * Setup operations to be performed after dvs_cam_ has been loaded
     *
     *     ROS params:
     *         virtual_width) voxel grid width
     *         virtual_height) voxel grid height
     *         fov_virtual_camera_deg) voxel grid fov
     */
    void postCameraLoaded();
};

#endif  // TRACKER_H
