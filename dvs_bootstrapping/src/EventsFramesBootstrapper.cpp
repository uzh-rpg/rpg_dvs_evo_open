#include "dvs_bootstrapping/EventsFramesBootstrapper.hpp"

#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

// #include "evo_utils/geometry.hpp"
#include <opencv2/video/tracking.hpp>

#include "evo_utils/geometry.hpp"
#include "rpg_common_ros/params_helper.hpp"

namespace dvs_bootstrapping {

EventsFramesBootstrapper::~EventsFramesBootstrapper() {}
EventsFramesBootstrapper::EventsFramesBootstrapper(ros::NodeHandle &nh,
                                                   ros::NodeHandle &nhp)
    : Bootstrapper(nh, nhp), it_(nh) {
    postCameraLoaded();

    rate_hz_ = rpg_common_ros::param<int>(nhp, "rate_hz", 25);
    frame_size_ = rpg_common_ros::param<int>(nhp, "frame_size", 50000);
    local_frame_size_ =
        rpg_common_ros::param<int>(nhp, "local_frame_size", 10000);
    CHECK_GE(frame_size_, local_frame_size_);
    enable_visuals_ =
        rpg_common_ros::param<bool>(nhp, "enable_visualizations", false);
    if (enable_visuals_) {
        pub_event_img_ = it_.advertise(
            rpg_common_ros::param<std::string>(nhp, "motion_corrected_topic",
                                               "/evo/bootstrap/event_frame"),
            1);
        pub_optical_flow_ = it_.advertise(
            rpg_common_ros::param<std::string>(nhp, "optical_flow_topic",
                                               "/evo/bootstrap/optical_flow"),
            1);
    }

    // Events must be processed quickly
    // Hence, we store and process them in two separate threads
    std::thread integrate(&EventsFramesBootstrapper::integratingThread, this);
    integrate.detach();
}

void EventsFramesBootstrapper::postCameraLoaded() {
    static int nIt =
        rpg_common_ros::param<int>(nhp_, "unwarp_estimate_n_it", 50);
    static double eps =
        rpg_common_ros::param<double>(nhp_, "unwarp_estimate_eps", 1e-2);
    static int lvls =
        rpg_common_ros::param<int>(nhp_, "unwarp_estimate_pyramid_lvls", 2);

    std::lock_guard<std::mutex> lock(data_mutex_);
    sensor_size_ = cam_.fullResolution();
    height_ = sensor_size_.height;
    width_ = sensor_size_.width;

    // pre compute rectification table and weights
    cv::initUndistortRectifyMap(cam_.intrinsicMatrix(), cam_.distortionCoeffs(),
                                cv::noArray(), cv::noArray(), sensor_size_,
                                CV_32FC1, undistort_mapx_, undistort_mapy_);

    unwarp_params_ = motion_correction::WarpUpdateParams(
        nIt, eps, cv::MOTION_HOMOGRAPHY, lvls, sensor_size_);
    eventQueue_.clear();

    // Precompute rectification table
    evo_utils::camera::precomputeRectificationTable(rectified_points_, cam_);
}

void EventsFramesBootstrapper::integratingThread() {
    static ros::Rate r(rate_hz_);

    while (ros::ok()) {
        r.sleep();

        if (!idle_) {
            if (integrateEvents()) {
                // pauseBootstrapper();
                // bootstrap();
                clearEventQueue();
            }
        }
    }
}

void EventsFramesBootstrapper::clearEventQueue() {
    if (newest_processed_event_ <= frame_size_) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    eventQueue_.erase(
        eventQueue_.begin(),
        eventQueue_.begin() + (newest_processed_event_ - frame_size_));
}

bool EventsFramesBootstrapper::integrateEvents() {
    static cv::Mat img0, img1, warp, event_img;
    static cv::Mat flow_field;
    static std::vector<dvs_msgs::Event> events;
    static size_t min_step_size =
        rpg_common_ros::param<int>(nhp_, "min_step_size", 5000);
    static float events_scale_factor =
        rpg_common_ros::param<float>(nhp_, "events_scale_factor", 7);
    static size_t th_min =
        rpg_common_ros::param<float>(nhp_, "activation_threshold_min", 200);
    static size_t th_patch_size =
        rpg_common_ros::param<int>(nhp_, "activation_threshold_patch_size", 3);
    static size_t median_filter_size =
        rpg_common_ros::param<int>(nhp_, "median_filter_size", 3);
    static bool median_filtering =
        rpg_common_ros::param<bool>(nhp_, "median_filtering", true);
    static bool adaptive_thresholding =
        rpg_common_ros::param<bool>(nhp_, "adaptive_thresholding", true);

    // perform this step in a thread safe manner
    std::lock_guard<std::mutex> lock_events(data_mutex_);
    if (eventQueue_.size() <= frame_size_) return false;

    size_t last_event = eventQueue_.size() - 1;
    if (last_event - newest_processed_event_ < min_step_size) return false;

    auto frame0_begin = eventQueue_.end() - frame_size_;
    auto frame0_end = frame0_begin + local_frame_size_;
    auto frame1_begin = eventQueue_.end() - local_frame_size_;
    auto frame1_end = eventQueue_.end() - 1;
    const double dt = (frame1_end->ts - frame0_begin->ts).toSec();

    motion_correction::resetMat(img0, sensor_size_, CV_32F);
    motion_correction::resetMat(img1, sensor_size_, CV_32F);
    motion_correction::resetMat(event_img, sensor_size_, CV_32F);

    motion_correction::drawEventsUndistorted(
        frame0_begin, frame0_end, img0, sensor_size_, rectified_points_, false);

    motion_correction::drawEventsUndistorted(
        frame1_begin, frame1_end, img1, sensor_size_, rectified_points_, false);

    static cv::Mat I33 = cv::Mat::eye(3, 3, CV_32F);
    warp = I33;

    motion_correction::updateWarp(warp, img0, img1, unwarp_params_);

    flow_field = motion_correction::computeFlowFromWarp(warp, dt, sensor_size_,
                                                        rectified_points_);

    motion_correction::drawEventsMotionCorrectedOpticalFlow(
        frame0_begin, frame1_end, flow_field, event_img, sensor_size_,
        rectified_points_, false);

    cv::Mat frame = (255.0 / events_scale_factor) * (event_img);
    frame.convertTo(frame, CV_8U);

    if (median_filtering)
        evo_utils::geometry::huangMedianFilter(
            frame, frame, cv::Mat(height_, width_, CV_8U, 1),
            median_filter_size);
    if (adaptive_thresholding)
        // cv::adaptiveThreshold(frame, frame, 255,
        // cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        //                       cv::THRESH_BINARY, th_patch_size, -th_min);
        cv::threshold(frame, frame, th_min, 255,
                      cv::ThresholdTypes::THRESH_TOZERO);

    ts_ = frame1_end->ts;
    if (enable_visuals_) {
        publishOpticalFlowVectors(flow_field);
        publishEventImage(frame, ts_);

        static const int MAX_IMAGES = rpg_common_ros::param<int>(
            nhp_, "max_events_frames_saved_to_file", 0);
        static const std::string format = rpg_common_ros::param<std::string>(
            nhp_, "events_frames_filename_format",
            "/media/user/data/example/%06d.bmp");
        static int cnt = 0;
        if (cnt < MAX_IMAGES) {
            static auto f = const_cast<char *>(format.c_str());
            static auto size = std::snprintf(nullptr, 0, f, cnt);
            std::string image_name(size + 1, '\0');
            std::sprintf(&image_name[0], f, cnt);
            cv::imwrite(image_name, frame);
            ++cnt;
        }
    }

    newest_processed_event_ = last_event;

    // save img for later use
    std::lock_guard<std::mutex> lock_frames(img_mutex_);
    if (keep_frames_) {
        frames_.push_back(frame);
    }

    return true;
}

void EventsFramesBootstrapper::publishEventImage(const cv::Mat &img,
                                                 const ros::Time &ts) {
    static cv_bridge::CvImage cv_event_image;

    cv_event_image.encoding = "mono8";
    if (pub_event_img_.getNumSubscribers() > 0) {
        img.convertTo(cv_event_image.image, CV_8U);

        auto aux = cv_event_image.toImageMsg();
        aux->header.stamp = ts;
        pub_event_img_.publish(aux);
    }
}

void EventsFramesBootstrapper::publishOpticalFlowVectors(
    const cv::Mat &flow_field) {
    if (pub_optical_flow_.getNumSubscribers() <= 0) return;

    static cv::Mat disp;
    motion_correction::resetMat(disp, sensor_size_, CV_8UC3);

    static const int step = 18;
    static const float scale = 0.04;

    cv::Point2f origin;
    cv::Point2f end;
    for (int y = 0; y < flow_field.rows; y += step) {
        for (int x = 0; x < flow_field.cols; x += step) {
            const cv::Vec2f &flow_vec = flow_field.at<cv::Vec2f>(y, x);

            origin.x = x;
            origin.y = y;
            end.x = origin.x + scale * flow_vec[0];
            end.y = origin.y + scale * flow_vec[1];

            cv::arrowedLine(disp, origin, end, cv::Scalar(0, 0, 255), 1, 8, 0,
                            0.15);
        }
    }

    static cv_bridge::CvImage cv_event_image;
    cv_event_image.encoding = "bgr8";
    cv_event_image.image = disp;
    auto aux = cv_event_image.toImageMsg();
    pub_optical_flow_.publish(aux);
}

}  // namespace dvs_bootstrapping