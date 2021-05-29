#include "dvs_depth_from_defocus/depth_defocus_node.hpp"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

//#define DEBUG_SHOW_FAKE_FRAMES
// #define DEBUG_SHOW_FILTERED_DEPTHMAP

#include <cv_bridge/cv_bridge.h>
#include <dvs_slam_msgs/VoxelGrid.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#ifdef MAPPING_PERF
#include "evo_utils/main.hpp"
#endif

#include "rpg_common_ros/params_helper.hpp"

namespace depth_from_defocus {

using namespace evo_utils::geometry;
using namespace evo_utils::camera;

void DepthFromDefocusNode::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
    static bool got_camera_info = false;

    if (!got_camera_info) {
        dvs_cam_.fromCameraInfo(*msg);

        postCameraLoaded();

        // Currently, done only once
        got_camera_info = true;
    }
}

void DepthFromDefocusNode::postCameraLoaded() {
    cv::Size full_resolution = dvs_cam_.fullResolution();
    width_ = full_resolution.width;
    height_ = full_resolution.height;

    const float fov_virtual_cam_deg =
        rpg_common_ros::param<float>(nh_, "fov_virtual_camera_deg", 0.0);

    virtual_width_ = rpg_common_ros::param<int>(nh_, "virtual_width", width_);

    virtual_height_ =
        rpg_common_ros::param<int>(nh_, "virtual_height", height_);

    float f_virtual_cam_;

    if (fov_virtual_cam_deg == 0.) {
        f_virtual_cam_ = dvs_cam_.fx();
    } else {
        const float fov_virtual_cam_rad = fov_virtual_cam_deg * CV_PI / 180.0;
        f_virtual_cam_ =
            0.5 * (float)virtual_width_ / std::tan(0.5 * fov_virtual_cam_rad);
    }
    LOG(INFO) << "Focal length of virtual camera: " << f_virtual_cam_
              << " pixels";
    virtual_cam_ = PinholeCamera(
        virtual_width_, virtual_height_, f_virtual_cam_, f_virtual_cam_,
        0.5 * (float)virtual_width_, 0.5 * (float)virtual_height_);

    K_ << dvs_cam_.fx(), 0.f, dvs_cam_.cx(), 0.f, dvs_cam_.fy(), dvs_cam_.cy(),
        0.f, 0.f, 1.f;

    precomputeRectifiedPoints();
    resetMapper();
}

DepthFromDefocusNode::DepthFromDefocusNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
    const image_geometry::PinholeCameraModel& cam)
    : nh_(nh),
      pnh_(pnh),
      it_(nh),
      state_(IDLE),
      dvs_cam_(cam),
      pc_(new PointCloud),
      pc_global_(new PointCloud) {
    radius_search_ = rpg_common_ros::param<float>(pnh_, "radius_search", 0.05);
    min_num_neighbors_ =
        rpg_common_ros::param<int>(pnh_, "min_num_neighbors", 3);

    median_filter_size_ =
        rpg_common_ros::param<int>(pnh_, "median_filter_size", 7);

    regular_frame_id_ =
        rpg_common_ros::param(nh_, "dvs_frame_id", std::string("dvs"));
    bootstrap_frame_id_ =
        rpg_common_ros::param(nh_, "dvs_bootstrap_frame_id", regular_frame_id_);
    frame_id_ = bootstrap_frame_id_;
    world_frame_id_ =
        rpg_common_ros::param<std::string>(nh_, "world_frame_id", "world");

    tf_ = std::make_shared<tf::Transformer>(true, ros::Duration(10000.0));

    tf_sub_ = nh_.subscribe("tf", 0, &DepthFromDefocusNode::tfCallback, this);
    event_sub_ = nh_.subscribe("events", 0,
                               &DepthFromDefocusNode::processEventArray, this);
    remote_key_ = nh_.subscribe("remote_key", 0,
                                &DepthFromDefocusNode::remoteKeyCallback, this);
    copilot_sub_ = nh_.subscribe("/evo/copilot_remote", 0,
                                 &DepthFromDefocusNode::copilotCallback, this);

    pub_pc_ = pnh_.advertise<PointCloud>("pointcloud", 1);
    pc_->header.frame_id = world_frame_id_;

    pub_pc_global_ = pnh_.advertise<PointCloud>("pointcloud_global", 1);
    pc_global_->header.frame_id = world_frame_id_;

    pub_voxel_grid_ = pnh_.advertise<dvs_slam_msgs::VoxelGrid>("voxel_grid", 1);

    events_to_recreate_kf_ =
        rpg_common_ros::param<int>(pnh_, "events_to_recreate_kf", 2000000);

    camera_info_sub_ = nh_.subscribe(
        "camera_info", 1, &DepthFromDefocusNode::cameraInfoCallback, this);

    postCameraLoaded();

    auto_trigger_ = rpg_common_ros::param<bool>(pnh_, "auto_trigger", false);
    if (auto_trigger_) state_ = MAPPING;
}

void DepthFromDefocusNode::copilotCallback(const std_msgs::Bool& msg) {
    frame_id_ = (msg.data) ? regular_frame_id_ : bootstrap_frame_id_;
    LOG(INFO) << "Mapping switched copilot to " << frame_id_;
}

void DepthFromDefocusNode::tfCallback(const tf::tfMessage::ConstPtr& tf_msg) {
    if (state_ == IDLE /* && !auto_trigger_*/) {
        return;
    }

    for (const geometry_msgs::TransformStamped& transform_stamped_msg :
         tf_msg->transforms) {
        tf::StampedTransform t;
        tf::transformStampedMsgToTF(transform_stamped_msg, t);
        tf_->setTransform(t);

        // LOG(INFO) << transform_stamped_msg.child_frame_id << " " << frame_id_
        //           << " " << world_frame_id_ << " "
        //           << transform_stamped_msg.header.frame_id;

        if (transform_stamped_msg.child_frame_id == frame_id_) {
            // if (state_ == IDLE && auto_trigger_) {
            //     state_ = MAPPING;
            //     return;  // there are no events collected yet
            // }

            while (newest_tracked_event_ < event_queue_.size() &&
                   event_queue_[newest_tracked_event_ + 1].ts <
                       transform_stamped_msg.header.stamp)
                ++newest_tracked_event_;

            // LOG(INFO) << newest_tracked_event_ << " " <<
            // events_to_recreate_kf_
            //           << " " << auto_trigger_;

            if (frame_id_ == bootstrap_frame_id_) {
                // keep bootstrap frame also as regular frame for future use
                t.child_frame_id_ = regular_frame_id_;
                tf_->setTransform(t);
            }

            if (auto_trigger_ && (newest_tracked_event_ - current_event_ >
                                  events_to_recreate_kf_)) {
                update();
                auto_trigger_ = false;
                frame_id_ = regular_frame_id_;
            }
        }
    }
}

void DepthFromDefocusNode::update() {
    resetVoxelGrid();
    processEventQueue(true);
    synthesizeAndPublishDepth();
    publishGlobalMap();

    last_kf_update_event_ = current_event_;
    clearEventQueue();
}

void DepthFromDefocusNode::resetMapper() {
    event_queue_.clear();
    tf_->clear();
    pc_->clear();
    pc_global_->clear();
    setupVoxelGrid();
    current_event_ = newest_tracked_event_ = last_kf_update_event_ = 0;
    T_ref_w_.setIdentity();
}

void DepthFromDefocusNode::remoteKeyCallback(
    const std_msgs::String::ConstPtr& msg) {
    std::string command_str = msg->data;
    LOG(INFO) << "Received command: " << command_str;

    switch (state_) {
        case MAPPING:
            if (command_str == "update") {
#ifdef MAPPING_PERF
                TIMER_START(t1);
#endif
                update();
#ifdef MAPPING_PERF
                TIMER_STOP(t1, t2, duration);
                LOG(INFO) << "[Map update] comp. time: " << duration
                          << " milliseconds";
#endif
            } else if (command_str == "reset") {
                LOG(INFO) << "Resetting mapper. Will continue to respond to "
                             "'update' requests";
                resetMapper();
            } else if (command_str == "disable_map_expansion") {
                LOG(INFO) << "Switching mapper to IDLE mode. Will stop "
                             "treating 'update' requests";
                resetMapper();
                state_ = IDLE;
            }
            break;

        case IDLE:
        default:
            if (command_str == "reset" ||
                command_str == "enable_map_expansion" ||
                command_str == "bootstrap") {
                state_ = MAPPING;
                if (command_str == "bootstrap") auto_trigger_ = true;
                LOG(INFO) << "Swiching to MAPPING mode";
            } else if (command_str == "update" || command_str == "switch") {
                LOG(INFO) << "Ignoring command '" << command_str
                          << "' because I am in IDLE mode";
            }
            break;
    }
}

void DepthFromDefocusNode::processEventQueue(bool reset) {
    static const size_t skip_batches_normal =
                            rpg_common_ros::param<int>(pnh_, "skip_batches", 0),
                        skip_batches_reset = rpg_common_ros::param<int>(
                            pnh_, "skip_batches_for_reset", 0),
                        max_event_rate = rpg_common_ros::param<int>(
                            pnh_, "max_event_rate", 10000000),
                        frame_size = rpg_common_ros::param<int>(
                            pnh_, "frame_size", 1024),
                        min_batch_size = rpg_common_ros::param<int>(
                            pnh_, "min_batch_size", 20000);

    // Performance measurement
    static const size_t perf_interval = 1000000;
    static size_t events_processed = 0;
    static double last_ts = event_queue_[current_event_].ts.toSec(),
                  time_evolved = 0;

    const size_t n_events = newest_tracked_event_ - current_event_;

    if (n_events < min_batch_size) return;

    size_t skip_batches = (reset) ? skip_batches_reset : skip_batches_normal;

    // Adapt skip_batches to max_event_rate
    const double duration = (event_queue_[newest_tracked_event_].ts -
                             event_queue_[current_event_].ts)
                                .toSec();
    const float event_rate = n_events / duration;
    if (!reset && event_rate > max_event_rate) {
        skip_batches = ceil(event_rate / max_event_rate) - 1;
        //    LOG(WARNING) << "Adapt skip_batches to: " << skip_batches;
    }

    static std::vector<Eigen::Vector4f> events;
    static std::vector<Eigen::Vector3f> centers;
    events.clear();
    centers.clear();

#ifdef MAPPING_PERF
    TIMER_START(t1);
#endif

    // Group events by packets events
    long num_events_processed = 0;

#ifdef DEBUG_SHOW_FAKE_FRAMES
    cv::Mat integrated_img = cv::Mat::zeros(height_, width_, CV_8U);
#endif

    while (current_event_ + (skip_batches + 1) * frame_size <
           newest_tracked_event_) {
        evo_utils::geometry::Transformation T_w_cur, T_ref_cur;
        getPoseAt(event_queue_[current_event_ + frame_size / 2].ts, T_w_cur);
        T_ref_cur = T_ref_w_ * T_w_cur;

        const evo_utils::geometry::Transformation T_cur_ref =
            T_ref_cur.inverse();
        const Eigen::Matrix3f R = T_cur_ref.getRotationMatrix().cast<float>();
        const Eigen::Vector3f t = T_cur_ref.getPosition().cast<float>();

        // Project the points on plane at distance z0
        const float z0 = raw_depths_vec_[0];
        Eigen::Matrix3f R_corr = R;
        R_corr *= z0;
        R_corr.col(2) += t;

        centers.push_back(-R.transpose() * t);

        Eigen::Matrix3f H_cur_ref = K_ * R_corr * virtual_cam_.Kinv_;

        Eigen::Matrix3f H_ref_cur = H_cur_ref.inverse();
        Eigen::Matrix4f H_ref_cur_4x4;
        H_ref_cur_4x4.block<3, 3>(0, 0) = H_ref_cur;
        H_ref_cur_4x4.col(3).setZero();
        H_ref_cur_4x4.row(3).setZero();

        for (size_t i = 0; i != frame_size; ++i) {
            const dvs_msgs::Event& e = event_queue_[current_event_];
            Eigen::Vector4f p;

            p.head<2>() = precomputed_rectified_points_.col(e.y * width_ + e.x);
            p[2] = 1.;
            p[3] = 0.;

            p = H_ref_cur_4x4 * p;
            p /= p[2];
            p[2] = 1.;

            events.push_back(p);

            ++num_events_processed;
            ++current_event_;

#ifdef DEBUG_SHOW_FAKE_FRAMES
            integrated_img.at<uchar>(e.y, e.x) = 255;
#endif
        }

        current_event_ += skip_batches * frame_size;

#ifdef DEBUG_SHOW_FAKE_FRAMES
        cv::imshow("Integrated events", integrated_img);
        cv::waitKey(10);
        integrated_img = cv::Mat::zeros(height_, width_, CV_8U);
#endif
    }

    if (centers.size() == 0) return;

    projectEventsToVoxelGrid(events, centers);

#ifdef MAPPING_PERF
    events_processed += n_events;
    {
        TIMER_STOP(t1, t2, duration);
        time_evolved += duration / 1000;  // from ms to s
    }

    if (events_processed > perf_interval) {
        double cur_ts = event_queue_[current_event_].ts.toSec(),
               time_processed = cur_ts - last_ts;
        LOG(INFO) << "  Events/s: " << (double)events_processed / time_evolved;
        LOG(INFO) << "Event rate: "
                  << (double)events_processed / time_processed;
        LOG(WARNING) << " RT factor: " << time_processed / time_evolved;

        last_ts = cur_ts;
        time_evolved = 0;
        events_processed = 0;
    }
#endif
}

void DepthFromDefocusNode::projectEventsToVoxelGrid(
    const std::vector<Eigen::Vector4f>& events,
    const std::vector<Eigen::Vector3f>& centers) {
    static const int N = 128;
    typedef Eigen::Array<float, N, 1> Arrayf;

    const size_t frame_size = events.size() / centers.size();
    const float z0 = raw_depths_vec_[0];

#pragma omp parallel for if (events.size() >= 20000)
    for (size_t layer = 0; layer < raw_depths_vec_.size(); ++layer) {
        const Eigen::Vector4f* pe = &events[0];
        Vote* pgrid =
            &ref_voxel_grid_[layer * virtual_width_ * virtual_height_];

        for (size_t frame = 0; frame != centers.size(); ++frame) {
            const Eigen::Vector3f& C = centers[frame];
            const float zi = (float)raw_depths_vec_[layer],
                        a = z0 * (zi - C[2]),
                        bx = (z0 - zi) * (C[0] * virtual_cam_.fx_ +
                                          C[2] * virtual_cam_.cx_),
                        by = (z0 - zi) * (C[1] * virtual_cam_.fy_ +
                                          C[2] * virtual_cam_.cy_),
                        d = zi * (z0 - C[2]);

            // Update voxel grid now, N events per iteration
            for (size_t batch = 0; batch != frame_size / N; ++batch) {
                Arrayf X, Y;
                for (size_t i = 0; i != N; ++i) {
                    X[i] = pe[i][0];
                    Y[i] = pe[i][1];
                }

                X = (X * a + bx) / d;
                Y = (Y * a + by) / d;

                for (size_t i = 0; i != N; ++i)
                    voteForCellBilinear(X[i], Y[i], pgrid);

                pe += N;
            }
        }
    }
}

void DepthFromDefocusNode::resetVoxelGrid() {
    std::fill(ref_voxel_grid_.begin(), ref_voxel_grid_.end(), 0);

    evo_utils::geometry::Transformation T_w_cur;

    ros::Time last_stamp = event_queue_[newest_tracked_event_].ts;

    current_event_ = (newest_tracked_event_ > events_to_recreate_kf_)
                         ? newest_tracked_event_ - events_to_recreate_kf_
                         : 0u;

    // Center the voxel grid in the middle pose of the sequence of events
    // used to fill the DSI.
    getPoseAt(last_stamp, T_w_cur);

    LOG(INFO) << "Received a map creation request";
    LOG(INFO) << "Setting KF at time: " << last_stamp;

    T_ref_w_ = T_w_cur.inverse();
}

DepthFromDefocusNode::~DepthFromDefocusNode() {
    synthesizeAndPublishDepth();
    publishGlobalMap();
}

void DepthFromDefocusNode::setupVoxelGrid() {
    evo_utils::geometry::Depth min_depth =
        rpg_common_ros::param<float>(nh_, "min_depth", 1.0);
    evo_utils::geometry::Depth max_depth =
        rpg_common_ros::param<float>(nh_, "max_depth", 10.0);
    num_depth_cells_ =
        (size_t)rpg_common_ros::param<int>(nh_, "num_depth_cells", 64);

    adaptive_threshold_kernel_size_ =
        rpg_common_ros::param<int>(pnh_, "adaptive_threshold_kernel_size", 5);

    adaptive_threshold_c_ =
        rpg_common_ros::param<int>(pnh_, "adaptive_threshold_c", -10);

    CHECK_GT(adaptive_threshold_kernel_size_, 0);
    CHECK_EQ(adaptive_threshold_kernel_size_ % 2, 1);

    CHECK_GT(min_depth, 0.0);
    CHECK_GT(max_depth, min_depth);

    CHECK_LE(num_depth_cells_, 256u) << "Number of depth cells must be <= 256";

    //  depths_vec_ = LinearDepthVector(min_depth, max_depth, num_depth_cells_);
    depths_vec_ = InverseDepthVector(min_depth, max_depth, num_depth_cells_);
    raw_depths_vec_ = depths_vec_.getDepthVector();

    min_depth_ = min_depth;
    max_depth_ = max_depth;

    ref_voxel_grid_.resize(num_depth_cells_ * virtual_width_ * virtual_height_,
                           0);
}

void DepthFromDefocusNode::processEventArray(
    const dvs_msgs::EventArray::ConstPtr& event_array) {
    if (state_ == IDLE) {
        return;
    }

    for (const dvs_msgs::Event& e : event_array->events)
        event_queue_.push_back(e);

    // Add initial pose
    static bool sent_initial_pose = false;
    if (!sent_initial_pose) {
        tf::StampedTransform T(tf::Transform::getIdentity(), event_queue_[0].ts,
                               world_frame_id_, frame_id_);
        tf_->setTransform(T);
        sent_initial_pose = true;
    }
}

void DepthFromDefocusNode::clearEventQueue() {
    static size_t event_history_size_ = 5000000;

    size_t last_event = std::min(current_event_, last_kf_update_event_);

    if (last_event > event_history_size_) {
        size_t remove_events = last_event - event_history_size_;

        event_queue_.erase(event_queue_.begin(),
                           event_queue_.begin() + remove_events);
        current_event_ -= remove_events;
        last_kf_update_event_ -= remove_events;
        newest_tracked_event_ -= remove_events;
    }
}

void DepthFromDefocusNode::synthesizeAndPublishDepth() {
    //  LOG(INFO) << "Synthesize depth from voxel grid";
    cv::Mat depth, confidence;
    synthesizePointCloudFromVoxelGrid(depth, confidence);

    // Accumulate point cloud
    const evo_utils::geometry::Transformation T_w_ref = T_ref_w_.inverse();
    accumulatePointcloud(depth, confidence_mask_, confidence,
                         T_w_ref.getRotation().getRotationMatrix(),
                         T_w_ref.getPosition(),
                         event_queue_[current_event_].ts);

    publishVoxelGrid(T_w_ref);
    publishDepthmap(depth, confidence_mask_);
}

bool DepthFromDefocusNode::getPoseAt(const ros::Time& t,
                                     evo_utils::geometry::Transformation& T) {
    std::string* error_msg = new std::string();
    if (!tf_->canTransform(world_frame_id_, frame_id_, t, error_msg)) {
        LOG(WARNING) << t.toNSec() << " : " << *error_msg;
        return false;
    } else {
        tf::StampedTransform tr;
        tf_->lookupTransform(world_frame_id_, frame_id_, t, tr);

        tf::transformTFToKindr(tr, &T);

        return true;
    }
}

void DepthFromDefocusNode::precomputeRectifiedPoints() {
    precomputed_rectified_points_ = Eigen::Matrix2Xf(2, height_ * width_);
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; ++x) {
            cv::Point2d rectified_point =
                dvs_cam_.rectifyPoint(cv::Point2d(x, y));
            precomputed_rectified_points_.col(y * width_ + x) =
                Eigen::Vector2f(rectified_point.x, rectified_point.y);
        }
    }
}

/* // Generic way to try different focus measures, but slow.
void DepthFromDefocusNode::synthesizePointCloudFromVoxelGridContrast(cv::Mat&
depth_cell_indices, cv::Mat &confidence)
{
  // Bell-shaped mask for a patch
  const int half_patchsize = rpg_common_ros::param<int>(pnh_, "half_patchsize",
3); const int patch_size = 2 * half_patchsize + 1; cv::Mat gaussian_kernel =
cv::getGaussianKernel(patch_size, -1, CV_32F); cv::Mat gaussian_kernel_2d =
gaussian_kernel * gaussian_kernel.t();

  for(int y=half_patchsize; y<virtual_height_-half_patchsize; ++y)
  {
    for(int x=half_patchsize; x<virtual_width_-half_patchsize; ++x)
    {
      // Build vector containing the number of votes as a function of depth
      for(size_t depth_plane=0; depth_plane<num_depth_cells_; ++depth_plane)
      {
        Vote *pgrid = &ref_voxel_grid_[depth_plane * virtual_width_ *
virtual_height_]; cv::Mat slice_dsi(virtual_height_, virtual_width_, CV_32F,
pgrid);

        // Compute the contrast on a small patch around (x,y)
        cv::Scalar mean, stddev;
        cv::Mat patch = slice_dsi(cv::Rect(x-half_patchsize, y-half_patchsize,
patch_size, patch_size)).mul(gaussian_kernel_2d); cv::meanStdDev(patch, mean,
stddev);

        // Select maximum contrast along the optical ray
        if(stddev[0] > confidence.at<float>(y,x))
        {
          confidence.at<float>(y,x) = (float) stddev[0]; // max of contrast
along optical ray depth_cell_indices.at<uchar>(y,x) = depth_plane;
        }
      }
    }
  }
}
*/

void DepthFromDefocusNode::synthesizePointCloudFromVoxelGridContrast(
    cv::Mat& depth_cell_indices, cv::Mat& confidence) {
    // Bell-shaped mask for a patch
    const int half_patchsize =
        rpg_common_ros::param<int>(pnh_, "half_patchsize", 3);
    const int patch_size = 2 * half_patchsize + 1;

    // Local mean, mean square and standard deviation of a depth slice of the
    // DSI
    cv::Mat slice_local_mean, slice_local_MS, slice_local_var;

    for (size_t depth_plane = 0; depth_plane < num_depth_cells_;
         ++depth_plane) {
        // Get depth plane
        Vote* pgrid =
            &ref_voxel_grid_[depth_plane * virtual_width_ * virtual_height_];
        cv::Mat slice_dsi(virtual_height_, virtual_width_, CV_32F, pgrid);

        // Compute local variance (contrast) of current depth slice of the DSI
        cv::GaussianBlur(slice_dsi, slice_local_mean,
                         cv::Size(patch_size, patch_size), 0, 0);
        cv::GaussianBlur(slice_dsi.mul(slice_dsi), slice_local_MS,
                         cv::Size(patch_size, patch_size), 0, 0);
        slice_local_var =
            slice_local_MS - slice_local_mean.mul(slice_local_mean);

        // Retain maximum local variance (contrast) per pixel (i.e., per optical
        // ray)
        for (int y = half_patchsize; y < virtual_height_ - half_patchsize;
             ++y) {
            for (int x = half_patchsize; x < virtual_width_ - half_patchsize;
                 ++x) {
                // Compute the contrast on a small patch around (x,y)
                float contrast = slice_local_var.at<float>(y, x);

                // Select maximum contrast along the optical ray
                if (contrast > confidence.at<float>(y, x)) {
                    confidence.at<float>(y, x) =
                        contrast;  // max of contrast along optical ray
                    depth_cell_indices.at<uchar>(y, x) = depth_plane;
                }
            }
        }
    }

    // Return standard deviation instead of variance
    cv::sqrt(confidence, confidence);
}

void DepthFromDefocusNode::synthesizePointCloudFromVoxelGridGradMag(
    cv::Mat& depth_cell_indices, cv::Mat& confidence) {
    // Bell-shaped mask for a patch
    const int half_patchsize =
        rpg_common_ros::param<int>(pnh_, "half_patchsize", 3);
    const int patch_size = 2 * half_patchsize + 1;

    // Gradient of a depth slice of the DSI
    cv::Mat grad_x, grad_y, grad_mag;

    for (size_t depth_plane = 0; depth_plane < num_depth_cells_;
         ++depth_plane) {
        // Get depth plane
        Vote* pgrid =
            &ref_voxel_grid_[depth_plane * virtual_width_ * virtual_height_];
        cv::Mat slice_dsi(virtual_height_, virtual_width_, CV_32F, pgrid);

        // Compute local gradient magnitude (focus measure) of current depth
        // slice of the DSI
        cv::Sobel(slice_dsi, grad_x, CV_32F, 1, 0, patch_size);
        cv::Sobel(slice_dsi, grad_y, CV_32F, 0, 1, patch_size);
        grad_mag = grad_x.mul(grad_x) + grad_y.mul(grad_y);

        // Retain maximum local focus per pixel (i.e., per optical ray)
        for (int y = half_patchsize; y < virtual_height_ - half_patchsize;
             ++y) {
            for (int x = half_patchsize; x < virtual_width_ - half_patchsize;
                 ++x) {
                // Compute the focus measure on a small patch around (x,y)
                float focus = grad_mag.at<float>(y, x);

                // Select maximum focus value along the optical ray
                if (focus > confidence.at<float>(y, x)) {
                    confidence.at<float>(y, x) =
                        focus;  // max focus along optical ray
                    depth_cell_indices.at<uchar>(y, x) = depth_plane;
                }
            }
        }
    }

    // Return gradient magnitude instead of its square
    cv::sqrt(confidence, confidence);
}

void DepthFromDefocusNode::synthesizePointCloudFromVoxelGridLinf(
    cv::Mat& depth_cell_indices, cv::Mat& confidence) {
    // Storage for 1-D slice of the DSI (along optical ray), i.e.,
    // vector containing the number of votes as a function of depth
    std::vector<float> num_votes_vec(num_depth_cells_);

    for (int y = 0; y < virtual_height_; ++y) {
        for (int x = 0; x < virtual_width_; ++x) {
            // Build vector containing the number of votes as a function of
            // depth
            for (size_t depth_plane = 0; depth_plane < num_depth_cells_;
                 ++depth_plane) {
                num_votes_vec.at(depth_plane) =
                    voxelGridAt(ref_voxel_grid_, x, y, depth_plane);
            }

            // Look for inv_depth* = argmax(sharpness(inv_depth))
            auto max = std::max_element(std::begin(num_votes_vec),
                                        std::end(num_votes_vec));

            confidence.at<float>(y, x) =
                *max;  // max number of votes along optical ray
            depth_cell_indices.at<uchar>(y, x) =
                std::distance(std::begin(num_votes_vec), max);
        }
    }
}

void DepthFromDefocusNode::synthesizePointCloudFromVoxelGrid(
    cv::Mat& depth, cv::Mat& confidence) {
    confidence = cv::Mat::zeros(virtual_height_, virtual_width_, CV_32F);
    cv::Mat depth_cell_indices =
        cv::Mat::zeros(virtual_height_, virtual_width_, CV_8U);

    // Choose method to obtain depth and confidence maps from the DSI
    const int type_focus_measure =
        rpg_common_ros::param<int>(pnh_, "type_focus_measure", 0);

#ifdef MAPPING_PERF
    TIMER_START(t1);
#endif

    if (type_focus_measure == 1) {
        // LOG(INFO) << "Depth estimation by maximum local variance of DSI";
        // Maximum patch contrast (variance) along optical ray
        synthesizePointCloudFromVoxelGridContrast(depth_cell_indices,
                                                  confidence);
    } else if (type_focus_measure == 2) {
        // LOG(INFO) << "Depth estimation by maximum local gradient magnitude of
        // DSI"; Maximum patch focus along optical ray
        synthesizePointCloudFromVoxelGridGradMag(depth_cell_indices,
                                                 confidence);
    } else {
        // LOG(INFO) << "Depth estimation by maximum of DSI";
        // Maximum number of votes along optical ray
        synthesizePointCloudFromVoxelGridLinf(depth_cell_indices, confidence);
    }

#ifdef MAPPING_PERF
    TIMER_STOP(t1, t2, duration);
    LOG(INFO) << "[Focus measure] comp. time: " << duration << " milliseconds";
#endif

    // Adaptive thresholding to get the confidence map
    cv::Mat confidence_map;
    cv::normalize(confidence, confidence_map, 0.0, 255.0, cv::NORM_MINMAX);
    confidence_map.convertTo(confidence_map, CV_8U);

    cv::adaptiveThreshold(confidence_map, confidence_mask_, 1,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,
                          adaptive_threshold_kernel_size_,
                          -adaptive_threshold_c_);

    //  LOG(INFO) << "Median filtering depth map...";
    cv::Mat depth_cell_indices_filtered;
    // rpg_common::Timer median_filtering_timer;
    // median_filtering_timer.start();
    evo_utils::geometry::huangMedianFilter(
        depth_cell_indices, depth_cell_indices_filtered, confidence_mask_,
        median_filter_size_);
    //  LOG(INFO) << "Done in " << median_filtering_timer.stop() << " s";

    depth = cv::Mat(virtual_height_, virtual_width_, CV_32F,
                    cv::Scalar(max_depth_));
    for (int y = 0; y < virtual_height_; ++y) {
        for (int x = 0; x < virtual_width_; ++x) {
            depth.at<float>(y, x) = depths_vec_.cellIndexToDepth(
                depth_cell_indices_filtered.at<uchar>(y, x));
        }
    }

#ifdef DEBUG_SHOW_FILTERED_DEPTHMAP
    cv::Mat norm_depth;
    cv::normalize(depth, norm_depth, 0.f, 1.f, cv::NORM_MINMAX);
    cv::imshow("Raw depthmap", norm_depth);

    // cv::Mat norm_filtered_depth;
    // cv::normalize(filtered_depth, norm_filtered_depth, 0.f, 1.f,
    //               cv::NORM_MINMAX);
    // cv::imshow("Filtered depthmap", norm_filtered_depth);
    cv::waitKey(0);
#endif
}

void DepthFromDefocusNode::accumulatePointcloud(
    const cv::Mat& depth, const cv::Mat& mask, const cv::Mat& confidence,
    const Eigen::Matrix3d R_world_ref, const Eigen::Vector3d t_world_ref,
    const ros::Time& timestamp) {
    pc_->clear();

    for (int y = 0; y < virtual_height_; ++y) {
        for (int x = 0; x < virtual_width_; ++x) {
            if (mask.at<uint8_t>(y, x) > 0) {
                BearingVector f_ref =
                    virtual_cam_.projectPixelTo3dRay(Keypoint(x, y));
                f_ref.normalize();
                Eigen::Vector3d xyz_world =
                    R_world_ref * (f_ref / f_ref[2] * depth.at<float>(y, x)) +
                    t_world_ref;

                pcl::PointXYZI p_world;
                p_world.x = xyz_world.x();
                p_world.y = xyz_world.y();
                p_world.z = xyz_world.z();
                p_world.intensity = 1.0 / p_world.z;
                pc_->push_back(p_world);
            }
        }
    }

    // filter point cloud to remove outliers
    PointCloud::Ptr cloud_filtered(new PointCloud);
    pcl::RadiusOutlierRemoval<PointType> outrem;
    outrem.setInputCloud(pc_);
    outrem.setRadiusSearch(radius_search_);
    outrem.setMinNeighborsInRadius(min_num_neighbors_);
    outrem.filter(*cloud_filtered);

    pc_->swap(*cloud_filtered);
    std_msgs::Header ros_header = pcl_conversions::fromPCL(pc_->header);
    ros_header.stamp = timestamp;
    pcl_conversions::toPCL(ros_header, pc_->header);

    sensor_msgs::PointCloud2::Ptr pc_to_publish(new sensor_msgs::PointCloud2);

    if (!pc_->empty()) {
        pcl::toROSMsg(*pc_, *pc_to_publish);
        pc_to_publish->header.stamp = timestamp;
        pub_pc_.publish(pc_to_publish);
        //    std::cout << "Publish pointcloud (" << pc_->size() << " points)"
        //    << std::endl;
    }

    // Print statistics
    //  Vote max = 0;
    //  int count = 0;
    //  float avg = 0;
    //  for (const Vote &v: ref_voxel_grid_)
    //  {
    //    if (v > 0)
    //    {
    //      avg += v;
    //      ++count;
    //    }
    //    if (v > max)
    //      max = v;
    //  }
    //  avg /= count;
    //  LOG(INFO) << "VoxelGrid: max: " << max << ", avg: " << avg << std::endl;
}

void DepthFromDefocusNode::publishDepthmap(const cv::Mat& depth,
                                           const cv::Mat& mask) {
    static image_transport::Publisher pub =
        it_.advertise("/dvs_mapping/depthmap", 1);

    if (pub.getNumSubscribers() == 0) return;

    double min, max;
    int min_idx, max_idx;
    cv::minMaxIdx(depth, &min, &max, &min_idx, &max_idx, mask);

    cv::Mat img = 255.f * (depth - min) / (max - min);
    img.convertTo(img, CV_8U);
    cv::applyColorMap(img, img, cv::COLORMAP_JET);
    img.setTo(255, 1 - mask);
    //  cv::GaussianBlur(img, img, cv::Size(3,3), 0);

    std_msgs::Header header;
    header.stamp = event_queue_[current_event_].ts;
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    pub.publish(msg);
}

void DepthFromDefocusNode::publishVoxelGrid(
    const evo_utils::geometry::Transformation& T_w_ref) const {
    if (pub_voxel_grid_.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray array;

        array.layout.dim.push_back(std_msgs::MultiArrayDimension());
        array.layout.dim.push_back(std_msgs::MultiArrayDimension());
        array.layout.dim.push_back(std_msgs::MultiArrayDimension());

        array.layout.data_offset = (uint32_t)0;

        array.layout.dim[0].label = "height";
        array.layout.dim[0].size = virtual_height_;
        array.layout.dim[0].stride =
            num_depth_cells_ * virtual_height_ * virtual_width_;

        array.layout.dim[1].label = "width";
        array.layout.dim[1].size = virtual_width_;
        array.layout.dim[1].stride = num_depth_cells_ * virtual_width_;

        array.layout.dim[2].label = "depth";
        array.layout.dim[2].size = num_depth_cells_;
        array.layout.dim[2].stride = num_depth_cells_;

        for (int y = 0; y < virtual_height_; ++y) {
            for (int x = 0; x < virtual_width_; ++x) {
                for (size_t k = 0; k < num_depth_cells_; ++k) {
                    array.data.push_back(voxelGridAt(ref_voxel_grid_, x, y, k));
                }
            }
        }

        geometry_msgs::Pose T_w_ref_msg;
        dvs_slam_msgs::VoxelGrid voxel_grid_msg;

        geometry_msgs::Point pos;
        geometry_msgs::Quaternion quat;
        pos.x = T_w_ref.getPosition()[0];
        pos.y = T_w_ref.getPosition()[1];
        pos.z = T_w_ref.getPosition()[2];
        quat.x = T_w_ref.getRotation().x();
        quat.y = T_w_ref.getRotation().y();
        quat.z = T_w_ref.getRotation().z();
        quat.w = T_w_ref.getRotation().w();
        T_w_ref_msg.position = pos;
        T_w_ref_msg.orientation = quat;
        voxel_grid_msg.voxel_grid = array;
        voxel_grid_msg.T_w_ref = T_w_ref_msg;
        pub_voxel_grid_.publish(voxel_grid_msg);
    }
}

void DepthFromDefocusNode::publishGlobalMap() {
    static const int accumulate_once_every =
        rpg_common_ros::param<int>(pnh_, "accumulate_local_map_once_every", 1);
    static const int skip_first =
        rpg_common_ros::param<int>(pnh_, "global_point_cloud_skip_first", 0);
    static int accumulation_cnt = 0;
    static int skipped = 0;

    if (pub_pc_global_.getNumSubscribers() > 0 && skipped >= skip_first &&
        (accumulation_cnt % accumulate_once_every == 0)) {
        *pc_global_ += *pc_;
        pc_global_->header.stamp = pc_->header.stamp;

        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*pc_global_, *cloud);

        static const float voxel_filter_leaf_size =
            rpg_common_ros::param<float>(pnh_, "voxel_filter_leaf_size", .0f);

        if (voxel_filter_leaf_size > 0.0001f) {
            // apply voxel grid filter
            LOG(INFO) << "Global point cloud before filtering: "
                      << cloud->width * cloud->height << " data points ("
                      << pcl::getFieldsList(*cloud) << ").";

            // Create the filtering object
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud);

            sor.setLeafSize(voxel_filter_leaf_size, voxel_filter_leaf_size,
                            voxel_filter_leaf_size);
            sor.filter(*cloud_filtered);
            LOG(INFO) << "Global point cloud after filtering: "
                      << cloud_filtered->width * cloud_filtered->height
                      << " data points (" << pcl::getFieldsList(*cloud_filtered)
                      << ").";

            pcl::fromPCLPointCloud2(*cloud_filtered, *pc_global_);
        } else {
            pcl::fromPCLPointCloud2(*cloud, *pc_global_);
        }

        static const float radius_search =
            rpg_common_ros::param<float>(pnh_, "radius_search_global_map", .02);
        static const int min_num_neighbors =
            rpg_common_ros::param<int>(pnh_, "min_num_neighbors_global_map", 5);

        if (radius_search > 0) {
            // filter point cloud to remove outliers
            PointCloud::Ptr cloud_filtered(new PointCloud);
            pcl::RadiusOutlierRemoval<PointType> outrem;
            outrem.setInputCloud(pc_global_);
            outrem.setRadiusSearch(radius_search);
            outrem.setMinNeighborsInRadius(min_num_neighbors);
            outrem.filter(*cloud_filtered);

            pc_global_->swap(*cloud_filtered);
        }

        sensor_msgs::PointCloud2 pc_to_publish;
        pcl::toROSMsg(*pc_global_, pc_to_publish);
        pub_pc_global_.publish(pc_to_publish);
    }
    if (skipped++ >= skip_first)
        accumulation_cnt = (accumulation_cnt + 1) % accumulate_once_every;
}

}  // namespace depth_from_defocus
