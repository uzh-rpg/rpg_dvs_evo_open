#pragma once

#include <utility> // std::pair
#include <boost/shared_ptr.hpp>

// forward declarations
namespace gtsam {
class Values;
class Pose3;
class Point3;
class Cal3_S2;
template<typename KEY, typename VALUE> class FastMap;
template<class CALIBRATION> class SmartProjectionPoseFactor;
template<class POSE, class LANDMARK, class CALIBRATION> class GenericProjectionFactor;
class SmartProjectionParams;
} // namespace gtsam

namespace svo {

// Typedef of desired classes to reduce compile-time
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> ProjectionFactor;
typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;
typedef gtsam::FastMap<boost::shared_ptr<SmartFactor>, size_t> FactorIndexMap;
typedef gtsam::FastMap<int, std::pair<boost::shared_ptr<SmartFactor>, int>> SmartFactorMap;

} // namespace svo
