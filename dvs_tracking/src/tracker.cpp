#include "dvs_tracking/tracker.hpp"

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/EventArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_transport/image_transport.h>
#include <kindr/minimal/quat-transformation.h>
#include <nav_msgs/Path.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.hpp>
#include <thread>

#include "evo_utils/camera.hpp"
#include "evo_utils/main.hpp"
#include "rpg_common_ros/params_helper.hpp"

using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;

void Tracker::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    static bool got_camera_info = false;

    if (!got_camera_info) {
        c_.fromCameraInfo(*msg);

        postCameraLoaded();

        // Currently, done only once
        got_camera_info = true;
    }
}

void Tracker::postCameraLoaded() {
    width_ = c_.fullResolution().width;
    height_ = c_.fullResolution().height;
    fx_ = c_.fx();
    fy_ = c_.fy();
    cx_ = c_.cx();
    cy_ = c_.cy();
    rect_ = cv::Rect(0, 0, width_, height_);

    float fov = 2. * std::atan(c_.fullResolution().width / 2. / c_.fx());
    LOG(INFO) << "Field of view: " << fov / M_PI * 180.;

    new_img_ = cv::Mat(c_.fullResolution(), CV_32F, cv::Scalar(0));

    sensor_msgs::CameraInfo cam_ref = c_.cameraInfo();
    cam_ref.width = nh_.param("virtual_width", c_.fullResolution().width);
    cam_ref.height = nh_.param("virtual_height", c_.fullResolution().height);
    cam_ref.P[0 * 4 + 2] = cam_ref.K[0 * 3 + 2] = 0.5 * (float)cam_ref.width;
    cam_ref.P[1 * 4 + 2] = cam_ref.K[1 * 3 + 2] = 0.5 * (float)cam_ref.height;

    float f_ref = nh_.param("fov_virtual_camera_deg", 0.);
    if (f_ref == 0.)
        f_ref = c_.fx();
    else {
        const float f_ref_rad = f_ref * CV_PI / 180.0;
        f_ref = 0.5 * (float)cam_ref.width / std::tan(0.5 * f_ref_rad);
    }
    cam_ref.P[0 * 4 + 0] = cam_ref.K[0 * 3 + 0] = f_ref;
    cam_ref.P[1 * 4 + 1] = cam_ref.K[1 * 3 + 1] = f_ref;
    c_ref_.fromCameraInfo(cam_ref);

    reset();
}

Tracker::Tracker(ros::NodeHandle& nh, ros::NodeHandle nh_private)
    : nh_(nh),
      nhp_(nh_private),
      it_(nh_),
      tf_(true, ros::Duration(2.))

      ,
      cur_ev_(0),
      kf_ev_(0),
      noise_rate_(nhp_.param("noise_rate", 10000))

      ,
      frame_size_(nhp_.param("frame_size", 2500)),
      step_size_(nhp_.param("step_size", 2500))

      ,
      idle_(true) {
    batch_size_ = nhp_.param("batch_size", 500);
    max_iterations_ = nhp_.param("max_iterations", 100);
    map_blur_ = nhp_.param("map_blur", 5);

    pyramid_levels_ = nhp_.param("pyramid_levels", 1);

    weight_scale_trans_ =
        rpg_common_ros::param<float>(nhp_, "weight_scale_translation", 0.);
    weight_scale_rot_ =
        rpg_common_ros::param<float>(nhp_, "weight_scale_rotation", 0.);

    T_world_kf_ = T_kf_ref_ = T_ref_cam_ = T_cur_ref_ =
        Eigen::Affine3f::Identity();

    map_ = PointCloud::Ptr(new PointCloud);
    map_local_ = PointCloud::Ptr(new PointCloud);

    // Load camera calibration
    c_ = evo_utils::camera::loadPinholeCamera(nh);
    postCameraLoaded();

    // Setup Subscribers
    event_sub_ = nh_.subscribe("events", 0, &Tracker::eventCallback, this);
    map_sub_ = nh_.subscribe("pointcloud", 0, &Tracker::mapCallback, this);
    remote_sub_ =
        nh_.subscribe("remote_key", 0, &Tracker::remoteCallback, this);
    tf_sub_ = nh_.subscribe("tf", 0, &Tracker::tfCallback, this);
    camera_info_sub_ =
        nh_.subscribe("camera_info", 1, &Tracker::cameraInfoCallback, this);

    // Setup Publishers
    poses_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("evo/pose", 0);

#ifdef TRACKER_DEBUG_REFERENCE_IMAGE
    std::thread map_overlap(&Tracker::publishMapOverlapThread, this);
    map_overlap.detach();
#endif

    frame_id_ =
        rpg_common_ros::param(nh_, "dvs_frame_id", std::string("dvs_evo"));
    world_frame_id_ =
        rpg_common_ros::param(nh_, "world_frame_id", std::string("world"));
    auto_trigger_ = rpg_common_ros::param<bool>(nhp_, "auto_trigger", false);

    std::thread tracker(&Tracker::trackingThread, this);
    tracker.detach();
}

void Tracker::trackingThread() {
    static ros::Rate r(100);

    LOG(INFO) << "Spawned tracking thread.";

    while (ros::ok()) {
        r.sleep();

        if (!idle_ && keypoints_.size() > 0) {
            estimateTrajectory();
        }
    }
}

void Tracker::remoteCallback(const std_msgs::String::ConstPtr& msg) {
    const std::string& cmd = msg->data;

    if (cmd == "switch")
        initialize(ros::Time(0));
    else if (cmd == "reset")
        reset();
    else if (cmd == "bootstrap")
        auto_trigger_ = true;
}

void Tracker::tfCallback(const tf::tfMessagePtr& msgs) {
    if (!idle_) return;

    for (auto& msg : msgs->transforms) {
        tf::StampedTransform t;
        tf::transformStampedMsgToTF(msg, t);
        tf_.setTransform(t);
    }
}

void Tracker::initialize(const ros::Time& ts) {
    std::string bootstrap_frame_id = rpg_common_ros::param<std::string>(
        nh_, "dvs_bootstrap_frame_id", std::string("/camera0"));
    tf::StampedTransform TF_kf_world;
    Eigen::Affine3d T_kf_world;
    tf_.lookupTransform(bootstrap_frame_id, world_frame_id_, ts, TF_kf_world);
    tf::transformTFToEigen(TF_kf_world, T_kf_world);

    T_world_kf_ = T_kf_world.cast<float>().inverse();
    T_kf_ref_ = Eigen::Affine3f::Identity();
    T_ref_cam_ = Eigen::Affine3f::Identity();

    while (cur_ev_ + 1 < events_.size() &&
           events_[cur_ev_].ts < TF_kf_world.stamp_)
        ++cur_ev_;

    updateMap();

    idle_ = false;
}

void Tracker::reset() {
    idle_ = true;

    events_.clear();
    poses_.clear();
    poses_filtered_.clear();
    cur_ev_ = kf_ev_ = 0;
}

void Tracker::eventCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
    static const bool discard_events_when_idle =
        rpg_common_ros::param<bool>(nhp_, "discard_events_when_idle", false);

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (discard_events_when_idle && idle_) return;

    clearEventQueue();
    for (const auto& e : msg->events) events_.push_back(e);
}

void Tracker::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    static size_t min_map_size =
        rpg_common_ros::param<int>(nhp_, "min_map_size", 0);

    std::lock_guard<std::mutex> lock(data_mutex_);

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *map_);

    LOG(INFO) << "Received new map: " << map_->size() << " points";

    if (map_->size() > min_map_size && auto_trigger_) {
        LOG(INFO) << "Auto-triggering tracking";

        // initialize(msg->header.stamp);
        initialize(ros::Time(0));
        auto_trigger_ = false;
    }
}

void Tracker::updateMap() {
    static size_t min_map_size =
        rpg_common_ros::param<int>(nhp_, "min_map_size", 0);
    static size_t min_n_keypoints =
        rpg_common_ros::param<int>(nhp_, "min_n_keypoints", 0);

    if (map_->size() <= min_map_size) {
        LOG(WARNING) << "Unreliable map! Can not update map.";
        return;
    }

    T_kf_ref_ = T_kf_ref_ * T_ref_cam_;
    T_ref_cam_ = Eigen::Affine3f::Identity();
    kf_ev_ = cur_ev_;

    projectMap();

    if (keypoints_.size() < min_n_keypoints) {
        LOG(WARNING) << "Losing track!";
        // TODO: do something about it
    }
}

void Tracker::clearEventQueue() {
    static size_t event_history_size_ = 500000;

    if (idle_) {
        if (events_.size() > event_history_size_) {
            events_.erase(events_.begin(), events_.begin() + events_.size() -
                                               event_history_size_);

            cur_ev_ = kf_ev_ = 0;
        }
    } else {
        events_.erase(events_.begin(), events_.begin() + kf_ev_);

        cur_ev_ -= kf_ev_;
        kf_ev_ = 0;
    }
}

void Tracker::publishMapOverlapThread() {
    static ros::Rate r(nhp_.param("event_map_overlap_rate", 25));
    static const float z0 = 1. / nh_.param("max_depth", 10.),
                       z1 = 1. / nh_.param("min_depth", .1), z_range = z1 - z0;

    static cv::Mat cmap;
    if (!cmap.data) {
        cv::Mat gray(256, 1, CV_8U);
        for (int i = 0; i != gray.rows; ++i) gray.at<uchar>(i) = i;
        cv::applyColorMap(gray, cmap, cv::COLORMAP_RAINBOW);
    }

    static image_transport::Publisher pub =
        it_.advertise("event_map_overlap", 1);

    cv::Mat ev_img, img;

    while (ros::ok()) {
        r.sleep();

        if (idle_ || pub.getNumSubscribers() == 0 || event_rate_ < noise_rate_)
            continue;

        cv::convertScaleAbs(1. - .25 * new_img_, ev_img, 255);
        cv::cvtColor(ev_img, img, cv::COLOR_GRAY2RGB);

        Eigen::Affine3f T_cam_w =
            (T_world_kf_ * T_kf_ref_ * T_ref_cam_).inverse();

        const int s = 2;

        for (const auto& P : map_local_->points) {
            Eigen::Vector3f p = T_cam_w * Eigen::Vector3f(P.x, P.y, P.z);
            p[0] = p[0] / p[2] * fx_ + cx_;
            p[1] = p[1] / p[2] * fy_ + cy_;

            int x = std::round(s * p[0]), y = std::round(s * p[1]);
            float z = p[2];

            if (x < 0 || x >= s * width_ || y < 0 || y >= s * height_) continue;

            cv::Vec3b c = cmap.at<cv::Vec3b>(
                std::min(255., std::max(255. * (1. / z - z0) / z_range, 0.)));
            cv::circle(img, cv::Point(x, y), 2, cv::Scalar(c[0], c[1], c[2]),
                       -1, cv::LINE_AA, 1);
        }

        std_msgs::Header header;
        header.stamp = events_[cur_ev_].ts;

        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        pub.publish(msg);
    }
}

void Tracker::publishTF() {
    Eigen::Affine3f T_world_cam = T_world_kf_ * T_kf_ref_ * T_ref_cam_;
    tf::Transform pose_tf;
    tf::transformEigenToTF(T_world_cam.cast<double>(), pose_tf);
    tf::StampedTransform new_pose(pose_tf, events_[cur_ev_ + frame_size_].ts,
                                  world_frame_id_, "dvs_evo_raw");
    poses_.push_back(new_pose);
    tf_pub_.sendTransform(new_pose);

    tf::StampedTransform filtered_pose;
    if (getFilteredPose(filtered_pose)) {
        filtered_pose.frame_id_ = world_frame_id_;
        filtered_pose.child_frame_id_ = frame_id_;
        tf_pub_.sendTransform(filtered_pose);
        poses_filtered_.push_back(filtered_pose);

        publishPose();
    }
}

void Tracker::publishPose() {
    const tf::StampedTransform& T_world_cam = poses_.back();

    const tf::Vector3& p = T_world_cam.getOrigin();
    const tf::Quaternion& q = T_world_cam.getRotation();
    geometry_msgs::PoseStampedPtr msg_pose(new geometry_msgs::PoseStamped);
    msg_pose->header.stamp = T_world_cam.stamp_;
    msg_pose->header.frame_id = frame_id_;
    msg_pose->pose.position.x = p.x();
    msg_pose->pose.position.y = p.y();
    msg_pose->pose.position.z = p.z();
    msg_pose->pose.orientation.x = q.x();
    msg_pose->pose.orientation.y = q.y();
    msg_pose->pose.orientation.z = q.z();
    msg_pose->pose.orientation.w = q.w();
    poses_pub_.publish(msg_pose);
}

bool Tracker::getFilteredPose(tf::StampedTransform& pose) {
    static const size_t mean_filter_size =
        nhp_.param("pose_mean_filter_size", 10);

    if (mean_filter_size < 2) {
        pose = poses_.back();
        return true;
    }

    if (poses_.size() < mean_filter_size) {
        return false;
    }

    static Eigen::VectorXd P(7);
    P.setZero();

    // Take the first rotation q0 as the reference
    // Then, for the remainders rotations qi, instead of
    // averaging directly the qi's, average the incremental rotations
    // q0^-1 * q_i (in the Lie algebra), and then get the original mean
    // rotation by multiplying the mean incremental rotation on the left
    // by q0.

    tf::Quaternion tf_q0 =
        poses_[poses_.size() - mean_filter_size].getRotation();
    const Quaternion q0(tf_q0.w(), tf_q0.x(), tf_q0.y(), tf_q0.z());
    const Quaternion q0_inv = q0.inverse();

    for (size_t i = poses_.size() - mean_filter_size; i != poses_.size(); ++i) {
        const tf::Quaternion& tf_q = poses_[i].getRotation();
        const Quaternion q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
        const Quaternion q_inc = q0_inv * q;

        const tf::Vector3& t = poses_[i].getOrigin();

        Transformation T(q_inc, Eigen::Vector3d(t.x(), t.y(), t.z()));

        P.head<6>() += T.log();
        P[6] += poses_[i].stamp_.toSec();
    }

    P /= mean_filter_size;
    Transformation T(Transformation::Vector6(P.head<6>()));

    const Eigen::Vector3d& t_mean = T.getPosition();
    const Quaternion q_mean = q0 * T.getRotation();

    tf::StampedTransform filtered_pose;
    filtered_pose.setOrigin(tf::Vector3(t_mean[0], t_mean[1], t_mean[2]));
    filtered_pose.setRotation(
        tf::Quaternion(q_mean.x(), q_mean.y(), q_mean.z(), q_mean.w()));
    filtered_pose.stamp_ = ros::Time(P[6]);

    pose = filtered_pose;
    return true;
}

void Tracker::estimateTrajectory() {
    static const size_t max_event_rate = nhp_.param("max_event_rate", 8000000),
                        events_per_kf = nhp_.param("events_per_kf", 100000);

    std::lock_guard<std::mutex> lock(data_mutex_);

    while (true) {
        if (cur_ev_ + std::max(step_size_, frame_size_) > events_.size()) break;

        if (cur_ev_ - kf_ev_ >= events_per_kf) updateMap();

        if (idle_) break;

        size_t frame_end = cur_ev_ + frame_size_;

        // Skip frame if event rate below noise rate
        double frameduration =
            (events_[frame_end].ts - events_[cur_ev_].ts).toSec();
        event_rate_ =
            std::round(static_cast<double>(frame_size_) / frameduration);
        if (event_rate_ < noise_rate_) {
            LOG(WARNING) << "Event rate below NOISE RATE. Skipping frame.";
            cur_ev_ += step_size_;
            continue;
        }
        if (event_rate_ > max_event_rate) {
            LOG(WARNING) << "Event rate above MAX EVENT RATE. Skipping frame.";
            cur_ev_ += step_size_;
            continue;
        }

        static size_t events_processed = 0, poses_generated = 0;
#ifdef TRACKING_PERF
        // Performance analysis
        static const size_t perf_interval = 2000000;
        static double time_elapsed = 0., last_ts = events_[cur_ev_].ts.toSec();

        if (events_processed > perf_interval) {
            double cur_ts = events_[cur_ev_].ts.toSec(),
                   time_processed = cur_ts - last_ts;

            LOG(INFO) << "   Poses/s: "
                      << static_cast<double>(poses_generated) / time_elapsed;
            LOG(INFO) << " Pose rate: "
                      << static_cast<double>(poses_generated) / time_processed;
            LOG(INFO) << "  Events/s: "
                      << static_cast<double>(events_processed) / time_elapsed;
            LOG(INFO) << "Event rate: "
                      << static_cast<double>(events_processed) / time_processed;
            LOG(WARNING) << " RT factor: " << time_processed / time_elapsed;

            events_processed = 0;
            poses_generated = 0;
            time_elapsed = 0;
            last_ts = cur_ts;
        }
        TIMER_START(t1);
#endif

        drawEvents(events_.begin() + cur_ev_, events_.begin() + frame_end,
                   new_img_);
        cv::buildPyramid(new_img_, pyr_new_, pyramid_levels_);
        trackFrame();

        T_ref_cam_ *= SE3::exp(-x_).matrix();

        publishTF();
        cur_ev_ += step_size_;

#ifdef TRACKING_PERF
        {
            TIMER_STOP(t1, t2, duration);
            time_elapsed += duration;
            LOG(INFO) << "Tracking trajectory required: " << duration << "ms";
        }
#endif

        events_processed += step_size_;
        ++poses_generated;
    }
}
