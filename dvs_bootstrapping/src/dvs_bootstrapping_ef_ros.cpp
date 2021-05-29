#include <ros/ros.h>

#include "dvs_bootstrapping/EventsFramesBootstrapper.hpp"
#include "evo_utils/main.hpp"

RPG_COMMON_MAIN {
    ros::init(argc, argv, "dvs_bootstrapping_ros");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    dvs_bootstrapping::EventsFramesBootstrapper node(nh, nh_private);
    ros::spin();

    return 0;
}
