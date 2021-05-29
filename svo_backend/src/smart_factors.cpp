#include <svo/backend/smart_factors_fwd.h>

#include <gtsam/base/FastMap.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

namespace gtsam {

// Explicitely instantiate the desired classes
template class GenericProjectionFactor<Pose3, Point3, Cal3_S2>;
template class SmartProjectionPoseFactor<Cal3_S2>;
template class FastMap<boost::shared_ptr<SmartProjectionPoseFactor<Cal3_S2>>, size_t>;
template class FastMap<int, std::pair<boost::shared_ptr<SmartProjectionPoseFactor<Cal3_S2>>, int>>;

} // end namespace gtsam
