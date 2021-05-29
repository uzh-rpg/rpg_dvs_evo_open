#include <ros/ros.h>
#include "evo_utils/main.hpp"

#include "dvs_bootstrapping/FrontoPlanarBootstrapper.hpp"

RPG_COMMON_MAIN {
    ros::init(argc, argv, "dvs_bootstrapping_ros");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    dvs_bootstrapping::FrontoPlanarBootstrapper node(nh, nh_private);
    ros::spin();

    return 0;
}
