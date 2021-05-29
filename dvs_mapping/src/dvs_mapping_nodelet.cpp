#include <camera_info_manager/camera_info_manager.h>
#include <dvs_depth_from_defocus/dvs_mapping_nodelet.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include "evo_utils/camera.hpp"
#include "rpg_common_ros/params_helper.hpp"

namespace depth_from_defocus {

void DvsMappingNodelet::onInit() {
    ros::NodeHandle nh(getNodeHandle());

    // Load camera calibration
    image_geometry::PinholeCameraModel cam =
        evo_utils::camera::loadPinholeCamera(nh);

    node_ = new DepthFromDefocusNode(nh, getPrivateNodeHandle(), cam);

    NODELET_INFO_STREAM("Initialized " << getName() << " nodelet.");
}

PLUGINLIB_EXPORT_CLASS(depth_from_defocus::DvsMappingNodelet, nodelet::Nodelet)

}  // namespace depth_from_defocus
