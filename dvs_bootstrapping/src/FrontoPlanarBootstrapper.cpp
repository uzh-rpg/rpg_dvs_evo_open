#include "dvs_bootstrapping/FrontoPlanarBootstrapper.hpp"

#include <pcl/filters/radius_outlier_removal.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "evo_utils/geometry.hpp"
#include "rpg_common_ros/params_helper.hpp"

namespace dvs_bootstrapping {

FrontoPlanarBootstrapper::~FrontoPlanarBootstrapper() {}
FrontoPlanarBootstrapper::FrontoPlanarBootstrapper(ros::NodeHandle &nh,
                                                   ros::NodeHandle &nhp)
    : EventsFramesBootstrapper(nh, nhp), pcl_(new PointCloud) {
    postCameraLoaded();
    start_ = rpg_common_ros::param<bool>(nhp_, "auto_trigger", false);

    // Events must be processed quickly
    // Hence, we store and process them in two separate threads
    std::thread boot(&FrontoPlanarBootstrapper::bootstrappingThread, this);
    boot.detach();

    pub_pc_ = nh_.advertise<PointCloud>("pointcloud", 1);
    pcl_->header.frame_id = world_frame_id_;
}

void FrontoPlanarBootstrapper::bootstrappingThread() {
    static ros::Rate r(rate_hz_);

    static const bool one_shot =
        rpg_common_ros::param<bool>(nhp_, "one_shot", false);
    static const int n_subscribers_to_wait =
        rpg_common_ros::param<int>(nhp_, "n_subscribers_to_wait", 1);

    idle_ = false;  // this way events frames are published before actually
                    // bootstrapping

    while (ros::ok()) {
        r.sleep();
        std::lock_guard<std::mutex> lock_frames(img_mutex_);
        if (start_) {
            keep_frames_ = true;
            if (!frames_.empty()) {
                if (pub_pc_.getNumSubscribers() >= n_subscribers_to_wait) {
                    // keep_frames_ = false;
                    if (bootstrap()) {
                        if (one_shot) {
                            // pauseBootstrapper();
                            start_ = false;
                        }
                    }
                }
            }
        } else if (frames_.size() > 1) {
            frames_.erase(frames_.begin(), frames_.end() - 2);
        }
    }
}

void FrontoPlanarBootstrapper::postBootstrapCalled() { start_ = true; }

bool FrontoPlanarBootstrapper::bootstrap() {
    static float plane_distance =
        rpg_common_ros::param<float>(nhp_, "plane_distance", 1.);

    static cv::Mat activation_mask;
    activation_mask = frames_.back();
    frames_.clear();
    pcl_->clear();

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (activation_mask.at<u_char>(y, x) > 0) {
                cv::Point3d f_ref = cam_.projectPixelTo3dRay(cv::Point2d(x, y));
                cv::Point3d p = f_ref * plane_distance / f_ref.z;

                // cv::Point3d p = plane_distance *

                PointType p_world;
                p_world.x = p.x;
                p_world.y = p.y;
                p_world.z = p.z;
                p_world.intensity = 1.0 / p_world.z;
                pcl_->push_back(p_world);
            }
        }
    }

    return publishPcl();
}

bool FrontoPlanarBootstrapper::publishPcl() {
    static const float radius_search =
        rpg_common_ros::param<float>(nhp_, "radius_search", 0.5);
    static const float min_num_neighbors =
        rpg_common_ros::param<int>(nhp_, "min_num_neighbors", 2);

    if (pcl_->empty()) {
        LOG(WARNING) << "Point cloud empty!";
        return false;
    }

    // filter point cloud to remove outliers
    PointCloud::Ptr cloud_filtered(new PointCloud);
    pcl::RadiusOutlierRemoval<PointType> outrem;
    outrem.setInputCloud(pcl_);
    outrem.setRadiusSearch(radius_search);
    outrem.setMinNeighborsInRadius(min_num_neighbors);
    outrem.filter(*cloud_filtered);
    pcl_->swap(*cloud_filtered);

    if (pcl_->empty()) {
        LOG(WARNING) << "Point cloud empty after filtering!";
        return false;
    }

    std_msgs::Header ros_header = pcl_conversions::fromPCL(pcl_->header);
    ros_header.stamp = ts_;

    tf::StampedTransform first_pose(tf::Transform::getIdentity(),
                                    ros_header.stamp, world_frame_id_,
                                    bootstrap_frame_id_);
    first_pose.child_frame_id_ = bootstrap_frame_id_;
    pub_tf_.sendTransform(first_pose);

    pcl_conversions::toPCL(ros_header, pcl_->header);

    sensor_msgs::PointCloud2::Ptr pc_to_publish(new sensor_msgs::PointCloud2);

    LOG(INFO) << "Fronto planar map published";
    pcl::toROSMsg(*pcl_, *pc_to_publish);
    pc_to_publish->header.stamp = ros_header.stamp;
    pub_pc_.publish(pc_to_publish);

    return true;
}

}  // namespace dvs_bootstrapping