#include <camera_info_manager/camera_info_manager.h>
#include <dvs_msgs/EventArray.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <map>
#include <opencv2/highgui/highgui.hpp>

#include "dvs_reconstruction/mosaic.hpp"
#include "evo_utils/camera.hpp"
#include "evo_utils/interpolation.hpp"
#include "evo_utils/main.hpp"
#include "rpg_common_ros/params_helper.hpp"

/**
 * Node reconstructing images from events
 *
 * @see Mosaic
 */
class DvsReconstruction {
   public:
    DvsReconstruction(ros::NodeHandle& nh, ros::NodeHandle nh_private);

   protected:
    /* ROS */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    tf::TransformListener tf_;  ///< listen to camera poses, "tf"
    std::string
        frame_id_;  ///< camera transform frame id, ROS param: "dvs_frame_id"

    ros::Subscriber event_sub_;           ///< listen to events, "events"
    ros::Subscriber camera_info_sub_;     ///< @see cameraInfoCallback
                                          ///< topic: "camera_info"
    std::deque<dvs_msgs::Event> events_;  ///< incoming events
    size_t events_for_reconstruction_;  ///< events required to reconstruct the
                                        ///< image, "events_for_reconstruction"

    ros::Subscriber map_sub_;                  ///< listen to map updates, "map"
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_;  ///< last published map

    Mosaic mosaic_;

    image_geometry::PinholeCameraModel c_;  ///< camera model

    size_t cur_ev_ = 0;  ///< current event

    /**
     * Enqueue incoming events for later processing
     *
     * @param msg incoming events
     */
    void eventCallback(const dvs_msgs::EventArray::ConstPtr& msg);

    /**
     * Update camera info on first received message
     *
     * @param msg Message containing camera info
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    /**
     * Update current stored map and perform reconstruction
     *
     * @param msg updated map
     *
     * @see doReconstruction
     */
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    /**
     * Compute the reconstruction using at most events_for_reconstruction_
     * events.
     *
     * @see Mosaic::compute
     */
    void doReconstruction();

   private:
    std::string world_frame_id_;  ///< The root frame id of the tf system
};

void DvsReconstruction::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
    static bool got_camera_info = false;

    if (!got_camera_info) {
        c_.fromCameraInfo(*msg);
        mosaic_.setCamModel(c_);

        // Currently, done only once
        got_camera_info = true;
    }
}

DvsReconstruction::DvsReconstruction(ros::NodeHandle& nh,
                                     ros::NodeHandle nh_private)
    : nh_(nh),
      nhp_(nh_private),
      tf_(ros::Duration(1000.)),

      map_(new pcl::PointCloud<pcl::PointXYZ>),
      mosaic_(rpg_common_ros::param<float>(nh_private, "sigma_m", 100.),
              rpg_common_ros::param<float>(nh_private, "init_cov", 10.),
              rpg_common_ros::param<int>(nh_private, "window_size", 5000),
              rpg_common_ros::param<int>(nh_private, "map_blur", 5), tf_, nh_,
              nhp_) {
    // Load camera calibration
    c_ = evo_utils::camera::loadPinholeCamera(nh);
    mosaic_.setCamModel(c_);

    frame_id_ = rpg_common_ros::param(nh_, "dvs_frame_id", std::string("dvs"));

    // Setup Subscribers
    event_sub_ =
        nh_.subscribe("events", 0, &DvsReconstruction::eventCallback, this);
    map_sub_ = nh_.subscribe("map", 0, &DvsReconstruction::mapCallback, this);
    camera_info_sub_ = nh_.subscribe(
        "camera_info", 1, &DvsReconstruction::cameraInfoCallback, this);
    world_frame_id_ =
        rpg_common_ros::param<std::string>(nh_, "world_frame_id", "world");
}

void DvsReconstruction::eventCallback(
    const dvs_msgs::EventArray::ConstPtr& msg) {
    for (const dvs_msgs::Event& e : msg->events) events_.push_back(e);
}

void DvsReconstruction::mapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *map);

    map_ = map;

    doReconstruction();
}

void DvsReconstruction::doReconstruction() {
    static const size_t events_for_reconstruction =
        nhp_.param("events_for_reconstruction", 100000);

    // Advance event index
    ros::Time t = pcl_conversions::fromPCL(map_->header.stamp);
    while (cur_ev_ < events_.size() && events_[cur_ev_].ts < t) ++cur_ev_;

    // Clear event queue
    if (cur_ev_ > events_for_reconstruction) {
        size_t remove_events = cur_ev_ - events_for_reconstruction;

        events_.erase(events_.begin(), events_.begin() + remove_events);
        cur_ev_ -= remove_events;
    }

    if (!tf_.canTransform(world_frame_id_, frame_id_, events_[0].ts) ||
        !tf_.canTransform(world_frame_id_, frame_id_, events_[cur_ev_].ts))
        return;

    std::vector<dvs_msgs::Event> evs(events_.begin(),
                                     events_.begin() + cur_ev_);
    mosaic_.compute(evs, map_);
}

RPG_COMMON_MAIN {
    ros::init(argc, argv, "dvs_reconstruction");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    DvsReconstruction test(nh, nh_private);
    ros::spin();

    return 0;
}
