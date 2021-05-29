// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#pragma once

#include <svo/common/types.h>
#include <svo/common/feature_wrapper.h>

namespace vk {
class AbstractCamera;
}

namespace svo {

class Matcher;
class OccupandyGrid2D;

/// Reprojector config parameters
/// Parameters marked with (!) are more important than the others.
struct ReprojectorOptions
{
  /// (!) Maximum numbers of features to match. The image is divided in a grid
  /// and we try to find at maximum one feature per cell to assure that the
  /// features are well distributed in the image. (-1) means unlimited.
  size_t max_n_features_per_frame = 120;

  /// (!) Cell width of a grid-cell. Controls the distribution of features.
  size_t cell_size = 30;

  /// We try to find the max_n_kfs closest keyframes that have overlapping
  /// field of view.
  size_t max_n_kfs = 5;

  /// If we don't find enough 3d points or converged seeds, also try to match
  /// unconverged seeds.
  bool reproject_unconverged_seeds = true;

  /// Remove points that have less than two observations
  /// Disable this flag when initializing 3D points from the ground truth
  bool remove_unconstrained_points = true;

  /// use affine transformation to compensate for brightness change
  bool affine_est_offset = true;
  bool affine_est_gain = false;
};

/// Project points from the map into the image and find the corresponding
/// feature (corner). We don't search a match for every point but only for one
/// point per cell. Thereby, we achieve a homogeneously distributed set of
/// matched features and at the same time we can save processing time by not
/// projecting all points.
class Reprojector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Reprojector> Ptr;

  Reprojector(const ReprojectorOptions& options,
              size_t camera_index);

  ~Reprojector() = default;

  ReprojectorOptions options_;

  struct Statistics
  {
    size_t n_matches = 0;
    size_t n_trials = 0;
  } stats_;

  /// A candidate is a point that projects into the image plane and for which we
  /// will search a maching feature in the image.
  struct Candidate
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FramePtr ref_frame;   //!< Reference frame.
    size_t ref_index;     //!< Feature index in reference frame.
    Keypoint cur_px;      //!< Projected 2D pixel location in current frame.
    int n_reproj = 0;     //!< Number of previously successful projections for quality.
    Score score;          //!< Feature Detection Score
    FeatureType type;     //!< Type of feature to determine quality.

    Candidate() = default;

    Candidate(const FramePtr _ref_frame, const size_t _ref_index,
              const Keypoint& _cur_px, const size_t _n_reproj,
              const Score _score, const FeatureType& _type)
      : ref_frame(_ref_frame)
      , ref_index(_ref_index)
      , cur_px(_cur_px)
      , n_reproj(_n_reproj)
      , score(_score)
      , type(_type)
    { ; }
  };
  using Candidates = std::vector<Candidate>;

  std::unique_ptr<OccupandyGrid2D> grid_;
  Candidates candidates_;
  size_t camera_index_; // When using multiple cameras, each camera has a reprojector.

  /// Project points seed in close_kfs into frame.
  void reprojectFrames(
      const FramePtr &frame,
      const std::vector<FramePtr>& close_kfs,
      std::vector<PointPtr>& trash_points);
};

namespace reprojector_utils {

  void sortCandidates(
    Reprojector::Candidates& candidates);

  void matchCandidates(const FramePtr& frame,
      const size_t max_n_features_per_frame,
      const bool affine_est_offset,
      const bool affine_est_gain,
      Reprojector::Candidates& candidates,
      OccupandyGrid2D& grid,
      Reprojector::Statistics& stats);

  bool matchCandidate(
      const FramePtr& frame,
      Reprojector::Candidate& c,
      Matcher& matcher,
      FeatureWrapper& feature);

  bool getCandidate(
      const FramePtr& cur_frame,
      const FramePtr& ref_frame,
      const size_t& ref_index,
      Reprojector::Candidate& candidate);

  bool projectPointAndCheckVisibility(
      const FramePtr& frame,
      const Eigen::Vector3d& xyz,
      Eigen::Vector2d* px);

  void setGridCellsOccupied(
      const Reprojector::Candidates& candidates,
      OccupandyGrid2D& grid);

} // namespace reprojector_utils
} // namespace svo
