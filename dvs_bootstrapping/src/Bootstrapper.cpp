#include "dvs_bootstrapping/Bootstrapper.hpp"

#include <camera_info_manager/camera_info_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <sensor_msgs/CameraInfo.h>
#include <string.h>

#include "rpg_common_ros/params_helper.hpp"

namespace dvs_bootstrapping {

Bootstrapper::Bootstrapper(ros::NodeHandle &nh, ros::NodeHandle &nhp)
    : nh_(nh), nhp_(nhp) {
    idle_ = !rpg_common_ros::param<bool>(nhp_, "auto_trigger", false);

    LOG(INFO) << "Bootstrapper initially idle: " << idle_;

    cam_ = evo_utils::camera::loadPinholeCamera(nh_);

    // subscribe / advertise
    event_sub_ = nh_.subscribe("events", 0, &Bootstrapper::eventCallback, this);
    remote_sub_ = nh_.subscribe("remote_key", 0,
                                &Bootstrapper::startCommandCallback, this);
    camera_info_sub_ = nh_.subscribe("camera_info", 1,
                                     &Bootstrapper::cameraInfoCallback, this);

    remote_key_pub_ = nh_.advertise<std_msgs::String>("remote_key", 1);

    world_frame_id_ = rpg_common_ros::param<std::string>(nh_, "world_frame_id",
                                                         std::string("world"));
    bootstrap_frame_id_ = rpg_common_ros::param(nh_, "dvs_bootstrap_frame_id",
                                                std::string("/dvs_evo"));
}

void Bootstrapper::startCommandCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "bootstrap") {
        // LOG(INFO) << "[Bootstrapper] received bootstrap command";
        // clean event queue
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            eventQueue_.clear();
        }

        LOG(INFO) << "Bootstrapping not idle anymore.";
        // set the state of the bootstrapper to active (might be used in
        // subclasses)
        idle_ = false;

        postBootstrapCalled();
    }
}

void Bootstrapper::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr &msg) {
    static bool got_camera_info = false;

    if (!got_camera_info) {
        cam_.fromCameraInfo(*msg);

        postCameraLoaded();

        // Currently, done only once
        got_camera_info = true;
    }
}

void Bootstrapper::pauseBootstrapper() {
    LOG(INFO) << "Bootstrapper is now idle.";
    idle_ = true;
}

void Bootstrapper::eventCallback(const dvs_msgs::EventArray::ConstPtr &msg) {
    if (idle_) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    eventQueue_.insert(eventQueue_.end(), msg->events.begin(),
                       msg->events.end());
}

}  // namespace dvs_bootstrapping