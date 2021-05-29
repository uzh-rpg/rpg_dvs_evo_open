#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/package.h>

#include <dvs_depth_from_defocus/depth_defocus_node.hpp>

#include "evo_utils/camera.hpp"
#include "evo_utils/geometry.hpp"
#include "evo_utils/main.hpp"
#include "rpg_common_ros/params_helper.hpp"

using namespace evo_utils::geometry;

RPG_COMMON_MAIN {
    ros::init(argc, argv, "dvs_mapping");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Load camera calibration
    image_geometry::PinholeCameraModel cam =
        evo_utils::camera::loadPinholeCamera(nh);

    depth_from_defocus::DepthFromDefocusNode depth_defocus_node(nh, pnh, cam);

    ros::spin();

    return 0;
}
