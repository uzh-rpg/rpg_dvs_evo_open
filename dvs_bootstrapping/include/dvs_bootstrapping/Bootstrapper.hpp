#pragma once

#ifndef BOOTSTRAPPING_HPP
#define BOOTSTRAPPING_HPP

#include <dvs_msgs/EventArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "evo_utils/camera.hpp"
// #include <deque>
#include <mutex>

namespace dvs_bootstrapping {

class Bootstrapper {
   public:
    Bootstrapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    virtual ~Bootstrapper() {}

    /**
     * Start bootstrap on command
     *
     * topic: "remote_key", message: "bootstrap"
     * When receives this message, reset data and set idle_ to false;
     *
     * @param msg Remote key message
     */
    void startCommandCallback(const std_msgs::String::ConstPtr &msg);

    /**
     * Update camera info on callback
     *
     * topic: "camera_info"
     *
     * @param msg Camera info msg
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    /**
     * Set idle_ to true and wait for "bootstrap" command
     *
     * @see startCommandCallback
     */
    void pauseBootstrapper();

   protected:
    /**
     * Setup operations to be performed after cam_ has been loaded
     */
    virtual void postCameraLoaded() {}

    /**
     * Setup operations to be performed after the bootstrap command has been
     * received.
     */
    virtual void postBootstrapCalled() {}

    /* ROS */
    ros::NodeHandle nh_, nhp_;
    image_geometry::PinholeCameraModel cam_;

    bool idle_ = true;  ///< Whether it is bootstrapping the pipeline

    std::vector<dvs_msgs::Event> eventQueue_;  ///< @see eventCallback
    std::mutex data_mutex_;  ///< to grant exclusive access to eventQueue_

    ros::Publisher remote_key_pub_;   ///< to trigger the rest of the
                                      ///< pipeline when needed
    std::string world_frame_id_;      ///< The root of the tf system
    std::string bootstrap_frame_id_;  ///< child frame id for bootstrap

   private:
    ros::Subscriber event_sub_;        ///< @see eventCallback
    ros::Subscriber remote_sub_;       ///< @see startCommandCallback
    ros::Subscriber camera_info_sub_;  ///< @see cameraInfoCalback

    /**
     * Accumulate incoming events for later processing
     *
     * @param msg Incoming event array
     */
    void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg);
};

}  // namespace dvs_bootstrapping

#endif