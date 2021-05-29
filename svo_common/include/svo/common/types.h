#pragma once

#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

namespace svo {

//------------------------------------------------------------------------------
// Scalars and fp precision.
using size_t = std::size_t;
using uint8_t = std::uint8_t;
using uint64_t = std::uint64_t;
using FloatType = double;

//------------------------------------------------------------------------------
// Feature containers.
using Keypoint = Eigen::Matrix<FloatType, 2, 1>;
using BearingVector = Eigen::Matrix<FloatType, 3, 1>;
using Position = Eigen::Matrix<FloatType, 3, 1>;
using GradientVector = Eigen::Matrix<FloatType, 2, 1>;
using SeedState = Eigen::Matrix<FloatType, 4, 1>;
using Level = int;
using Score = FloatType;
using Keypoints = Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor>;
using Bearings = Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Positions = Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Gradients = Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor>;
using Scores = Eigen::Matrix<FloatType, Eigen::Dynamic, 1, Eigen::ColMajor>;
using Levels = Eigen::Matrix<Level, Eigen::Dynamic, 1, Eigen::ColMajor>;
using InlierMask = Eigen::Matrix<bool, Eigen::Dynamic, 1, Eigen::ColMajor>;
using SeedStates = Eigen::Matrix<FloatType, 4, Eigen::Dynamic, Eigen::ColMajor>;
using TrackIds = Eigen::VectorXi;

//------------------------------------------------------------------------------
// Forward declarations and common types for simplicity.
struct Feature;
using FeaturePtr = std::shared_ptr<Feature>;
class Frame;
using FramePtr = std::shared_ptr<Frame>;
using FrameWeakPtr = std::weak_ptr<Frame>;
class FrameBundle;
using FrameBundlePtr = std::shared_ptr<FrameBundle>;
using BundleId = int;
using ImgPyr = std::vector<cv::Mat>;

//------------------------------------------------------------------------------
// Feature Type.
enum class FeatureType : uint8_t
{
  kEdgeletSeed = 0,
  kCornerSeed = 1,
  kEdgeletSeedConverged = 2,
  kCornerSeedConverged = 3,
  kEdgelet = 4,
  kCorner = 5,
  kOutlier = 6,
};
using FeatureTypes = std::vector<FeatureType>;

inline bool isSeed(const FeatureType& t)
{
  return static_cast<uint8_t>(t) < 4;
}

inline bool isConvergedSeed(const FeatureType& t)
{
  return (t == FeatureType::kEdgeletSeedConverged
          || t == FeatureType::kCornerSeedConverged);
}

inline bool isUnconvergedSeed(const FeatureType& t)
{
  return static_cast<uint8_t>(t) < 2;
}

inline bool isEdgelet(const FeatureType& t)
{
  return (t == FeatureType::kEdgelet
          || t == FeatureType::kEdgeletSeed
          || t == FeatureType::kEdgeletSeedConverged);
}

inline bool isCorner(const FeatureType& t)
{
  return (t == FeatureType::kCorner
          || t == FeatureType::kCornerSeed
          || t == FeatureType::kCornerSeedConverged);
}

} // namespace svo
