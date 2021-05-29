#pragma once

#include <nodelet/nodelet.h>

#include "dvs_depth_from_defocus/depth_defocus_node.hpp"

namespace depth_from_defocus {

class DvsMappingNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  DepthFromDefocusNode* node_;
};


}
