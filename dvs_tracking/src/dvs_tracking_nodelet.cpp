#include <pluginlib/class_list_macros.h>

#include "dvs_tracking/dvs_tracking_nodelet.h"

void DvsTrackingNodelet::onInit()
{
  node_ = new Tracker(getNodeHandle(), getPrivateNodeHandle());

  NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
}

PLUGINLIB_EXPORT_CLASS(DvsTrackingNodelet, nodelet::Nodelet)
