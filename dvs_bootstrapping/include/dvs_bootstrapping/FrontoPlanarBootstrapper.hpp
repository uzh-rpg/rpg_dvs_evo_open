#pragma once

#ifndef HARRIS_BOOTSTRAPPER_HPP
#define HARRIS_BOOTSTRAPPER_HPP

#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core.hpp>

#include "dvs_bootstrapping/EventsFramesBootstrapper.hpp"

namespace dvs_bootstrapping {

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

/**
 * Bootstrapper based on the EVO paper
 *
 * Assumes a fronto-planar scene and integrate a fixed number of events to
 * bootstrap the first map.
 *
 * Relies on EventsFramesBootstrapper for the events frames.
 */
class FrontoPlanarBootstrapper : public EventsFramesBootstrapper {
   public:
    FrontoPlanarBootstrapper(ros::NodeHandle &nh, ros::NodeHandle &nhp);
    virtual ~FrontoPlanarBootstrapper();

   protected:
    void postBootstrapCalled() override;

   private:
    ros::Publisher
        pub_pc_;           ///< publisher of the first approximation of the map
    PointCloud::Ptr pcl_;  ///< map

    tf::TransformBroadcaster pub_tf_;  ///< to send the first pose

    bool start_ = false;

    /**
     * Project aggregated event image onto a plane at a distance d_ from the
     * camera, and publish the point cloud as first map
     *
     *     ROS params:
     *         plane_distance) distance at which the events frame is reprojected
     *
     * @see publishPcl
     */
    bool bootstrap();

    /**
     * Regularly checks whether to bootstrap
     *
     *     ROS params:
     *         auto_trigger) if true, automatically starts publishing
     *              fronto-planar maps
     *         one_shot) if true, publishes only a single map before turning
     *              idle
     *         n_subscribers_to_wait) wait some subscribers before to publish
     *              the maps
     */
    void bootstrappingThread();

    /**
     * Publish pcl_ under the topic "pointcloud"
     *
     *     ROS params:
     *         radius_search) radius size of radius filter
     *         min_num_neighbors) minimum number of neighbors for radius
     *              filtering
     *
     * @return whether the point cloud was published
     */
    bool publishPcl();
};

}  // namespace dvs_bootstrapping

#endif