#pragma once

#include <nodelet/nodelet.h>

#include "dvs_tracking/tracker.hpp"

class DvsTrackingNodelet : public nodelet::Nodelet {
   public:
    virtual void onInit();

   private:
    Tracker* node_;
};
