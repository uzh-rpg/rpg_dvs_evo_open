#include "dvs_tracking/tracker.hpp"
#include "evo_utils/main.hpp"

RPG_COMMON_MAIN {
    ros::init(argc, argv, "dvs_tracking_ros");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Tracker node(nh, nh_private);
    ros::spin();

    return 0;
}
