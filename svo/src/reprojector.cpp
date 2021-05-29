#include <svo/reprojector.h>

#include <set>
#include <random> // std::mt19937
#include <algorithm>
#include <stdexcept>
#include <future>

#include <svo/common/camera.h>
#include <svo/common/point.h>
#include <svo/common/frame.h>
#include <svo/common/occupancy_grid_2d.h>
#include <svo/direct/matcher.h>
#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection_utils.h>
#include <vikit/math_utils.h>
#include <vikit/timer.h>

namespace svo {

Reprojector::Reprojector(
    const ReprojectorOptions& options,
    size_t camera_index)
  : options_(options)
  , camera_index_(camera_index)
{}

void Reprojector::reprojectFrames(
    const FramePtr& cur_frame,
    const std::vector<FramePtr>& visible_kfs,
    std::vector<PointPtr>& trash_points)
{
  CHECK_GT(options_.max_n_features_per_frame, 0u);
  cur_frame->resizeFeatureStorage(options_.max_n_features_per_frame);

  // Initialize grid
  if(!grid_)
  {
    grid_.reset(
          new OccupandyGrid2D(
            options_.cell_size,
            std::ceil(static_cast<double>(cur_frame->cam()->imageWidth())
                      / static_cast<double>(options_.cell_size)),
            std::ceil(static_cast<double>(cur_frame->cam()->imageHeight())
                      / static_cast<double>(options_.cell_size))));
  }
  grid_->reset();
  stats_.n_matches = 0;
  stats_.n_trials = 0;
  candidates_.clear();

  // Reproject all map points of the closest N kfs with overlap. We only store
  // in which grid cell the points fall.
  candidates_.clear();
  for(const FramePtr& ref_frame : visible_kfs)
  {
    // Try to reproject each map point that the other KF observes
    for(size_t i = 0; i < ref_frame->num_features_; ++i)
    {
      if(ref_frame->landmark_vec_[i] == nullptr
         || ref_frame->type_vec_[i] == FeatureType::kOutlier)
        continue;

      const PointPtr& point = ref_frame->landmark_vec_[i];

      // first check if the point is valid.
      if(point->n_failed_reproj_ > 10)
      {
        trash_points.push_back(point);
        continue;
      }

      // make sure we project a point only once
      if(point->last_projected_kf_id_.at(camera_index_) == cur_frame->id_)
        continue;
      point->last_projected_kf_id_[camera_index_] = cur_frame->id_;

      // make sure the point has at least two observations
      // ignore for the first 5 frames because in the benchmark node we may have
      // initialized some 3d points from the ground-truth.
      if(point->obs_.size() < 2 && options_.remove_unconstrained_points)
      {
        trash_points.push_back(point);
        continue;
      }

      // finally, reproject point
      Candidate candidate;
      if(reprojector_utils::getCandidate(cur_frame, ref_frame, i, candidate))
        candidates_.push_back(candidate);
    }
  }
  VLOG(5) << "There are " << candidates_.size() << " candidates from map points.";

  reprojector_utils::matchCandidates(
        cur_frame, options_.max_n_features_per_frame,
        options_.affine_est_offset, options_.affine_est_gain,
        candidates_, *grid_, stats_);

  VLOG(5) << "Reproject landmarks in cam-" << camera_index_
          << ": trials = " << stats_.n_trials << ", matches = " << stats_.n_matches;
  if(cur_frame->numFeatures() >= options_.max_n_features_per_frame
     && options_.max_n_features_per_frame > 0)
  {
    reprojector_utils::setGridCellsOccupied(candidates_, *grid_);
    // We don't return here because we want to set all the other grid cells occupied.
  }

  // ---------------------------------------------------------------------------
  // if we don't have enough features, we reproject the converged seeds
  candidates_.clear();
  for(const FramePtr& ref_frame : visible_kfs)
  {
    for(size_t i = 0; i < ref_frame->num_features_; ++i)
    {
      if(isConvergedSeed(ref_frame->type_vec_[i]))
      {
        Candidate candidate;
        if(reprojector_utils::getCandidate(cur_frame, ref_frame, i, candidate))
          candidates_.push_back(candidate);
      }
    }
  }
  if((cur_frame->numFeatures() >= options_.max_n_features_per_frame
      && options_.max_n_features_per_frame > 0))
  {
    // In this case we already have enough features but we just want to set the
    // occupancy such that the depth filter doesn't extract too many new features.
    // TODO(cfo): Actually, this should only be done if a new keyframe is selected.
    reprojector_utils::setGridCellsOccupied(candidates_, *grid_);
    candidates_.clear();
    return;
  }
  reprojector_utils::matchCandidates(
        cur_frame, options_.max_n_features_per_frame,
        options_.affine_est_offset, options_.affine_est_gain,
        candidates_, *grid_, stats_);
  VLOG(5) << "Reproject converged seeds in cam-" << camera_index_
          << ": trials = " << stats_.n_trials << ", matches = " << stats_.n_matches;
  if((cur_frame->numFeatures() >= options_.max_n_features_per_frame
      && options_.max_n_features_per_frame > 0)
     || !options_.reproject_unconverged_seeds)
  {
    reprojector_utils::setGridCellsOccupied(candidates_, *grid_);
    candidates_.clear();
    return;
  }

  // ---------------------------------------------------------------------------
  // if we still don't have enough features, reproject the unconverged seeds
  candidates_.clear();
  for(const FramePtr& ref_frame : visible_kfs)
  {
    for(size_t i = 0; i < ref_frame->num_features_; ++i)
    {
      if(isUnconvergedSeed(ref_frame->type_vec_[i]))
      {
        Candidate candidate;
        if(reprojector_utils::getCandidate(cur_frame, ref_frame, i, candidate))
          candidates_.push_back(candidate);
      }
    }
  }
  reprojector_utils::matchCandidates(
        cur_frame, options_.max_n_features_per_frame,
        options_.affine_est_offset, options_.affine_est_gain,
        candidates_, *grid_, stats_);
  VLOG(5) << "Reproject unconverged seeds in cam-" << camera_index_
          << ": trials = " << stats_.n_trials << ", matches = " << stats_.n_matches;
  if(cur_frame->numFeatures() >= options_.max_n_features_per_frame
     && options_.max_n_features_per_frame > 0)
  {
    reprojector_utils::setGridCellsOccupied(candidates_, *grid_);
    candidates_.clear();
    return;
  }
}

namespace reprojector_utils {

void sortCandidates(
    Reprojector::Candidates& candidates)
{
  std::sort(candidates.begin(), candidates.end(),
            [](const Reprojector::Candidate& lhs, const Reprojector::Candidate& rhs)
  {
    if(lhs.type > rhs.type
       || (lhs.type == rhs.type && lhs.n_reproj > rhs.n_reproj)
       || (lhs.type == rhs.type && lhs.n_reproj == rhs.n_reproj && lhs.score > rhs.score)) // TODO(check)
      return true;
    return false;
  });
}

void matchCandidates(
    const FramePtr& frame,
    const size_t max_n_features_per_frame,
    const bool affine_est_offset,
    const bool affine_est_gain,
    Reprojector::Candidates& candidates,
    OccupandyGrid2D& grid,
    Reprojector::Statistics& stats)
{
  sortCandidates(candidates);
  Matcher matcher;
  matcher.options_.affine_est_offset_ = affine_est_offset;
  matcher.options_.affine_est_gain_ = affine_est_gain;
  int i = 0;
  for(Reprojector::Candidate& candidate : candidates)
  {
    ++i;
    size_t grid_index =
        grid.getCellIndex(candidate.cur_px.x(),candidate.cur_px.y(), 1);
    if(max_n_features_per_frame > 0 && grid.isOccupied(grid_index))
      continue;

    ++stats.n_trials;
    FeatureWrapper feature_wrapper = frame->getEmptyFeatureWrapper();
    if(matchCandidate(frame, candidate, matcher, feature_wrapper))
    {
      ++stats.n_matches;
      ++frame->num_features_;
      grid.setOccupied(grid_index);
      if(max_n_features_per_frame > 0
         && frame->num_features_ >= max_n_features_per_frame)
        break;
    }
  }
  candidates.erase(candidates.begin(), candidates.begin()+i);
}

bool matchCandidate(
    const FramePtr& frame,
    Reprojector::Candidate& c,
    Matcher& matcher,
    FeatureWrapper& feature)
{
  GradientVector grad_ref;

  // direct matching
  CHECK_NOTNULL(c.ref_frame.get());
  CHECK_LT(c.ref_index, c.ref_frame->num_features_);
  int track_id = -1;
  if(c.ref_frame->landmark_vec_.at(c.ref_index) == nullptr)
  {
    FeatureWrapper ref_ftr = c.ref_frame->getFeatureWrapper(c.ref_index);
    if(isConvergedSeed(c.type))
    {
      const FloatType ref_depth = c.ref_frame->getSeedDepth(c.ref_index);
      Matcher::MatchResult res =
          matcher.findMatchDirect(*c.ref_frame, *frame, ref_ftr, ref_depth, c.cur_px);
      if(res != Matcher::MatchResult::kSuccess)
        return false;
    }
    else if(isUnconvergedSeed(c.type))
    {
      // TODO: make convergence threshold a parameter
      if(!depth_filter_utils::updateSeed(
           *frame, *c.ref_frame, c.ref_index, matcher, 200.0, false, false))
        return false;
    }
    else
      CHECK(false) << "Seed type unknown";

    grad_ref = ref_ftr.grad;
    track_id = ref_ftr.track_id;
    feature.seed_ref.keyframe = c.ref_frame;
    feature.seed_ref.seed_id = c.ref_index;
  }
  else
  {
    const PointPtr& point = c.ref_frame->landmark_vec_[c.ref_index];
    FramePtr ref_frame;
    size_t ref_feature_index;
    if(!point->getCloseViewObs(frame->pos(), ref_frame, ref_feature_index))
      return false;
    FeatureWrapper ref_ftr = ref_frame->getFeatureWrapper(ref_feature_index);
    CHECK_NOTNULL(ref_ftr.landmark.get()); // debug
    const FloatType ref_depth = (ref_frame->pos() - ref_ftr.landmark->pos()).norm();
    Matcher::MatchResult res = matcher.findMatchDirect(*ref_frame, *frame, ref_ftr, ref_depth, c.cur_px);
    if(res != Matcher::MatchResult::kSuccess)
    {
      point->n_failed_reproj_++;
      return false; // TODO(cfo): We should return match result and write in statistics.
    }
    point->n_succeeded_reproj_ += 1;
    grad_ref = ref_ftr.grad;
    feature.landmark = point;
    track_id = point->id();
  }


  // Set edgelet direction, check if is consistent.
  if(isEdgelet(c.type))
  {
    GradientVector g_predicted = (matcher.A_cur_ref_ * grad_ref).normalized();
    feature.grad = g_predicted;
    /*
    double angle = feature_detection_utils::getAngleAtPixelUsingHistogram(
          frame->img_pyr_[matcher.search_level_],
          (matcher.px_cur_ / (1<<matcher.search_level_)).cast<int>(), 4);
    feature.grad = GradientVector(std::cos(angle), std::sin(angle));

    constexpr double thresh = std::cos(30.0/180.0*3.81);
    if(std::abs(feature.grad.dot(g_predicted)) > thresh)
      return false;
    */
  }

  // Here we add a reference in the feature and frame to the 3D point, the other way
  // round is only done if this frame is selected as keyframe.
  feature.type = c.type;
  feature.px = matcher.px_cur_;
  feature.f = matcher.f_cur_;
  feature.level = matcher.search_level_;
  feature.track_id = track_id;
  feature.score = c.score;
  return true;
}

bool getCandidate(
    const FramePtr& cur_frame,
    const FramePtr& ref_frame,
    const size_t& ref_index,
    Reprojector::Candidate& candidate)
{
  Position xyz_world = Position::Zero();
  int n_reproj = 0;
  if(ref_frame->landmark_vec_[ref_index] != nullptr)
  {
    const PointPtr& point = ref_frame->landmark_vec_[ref_index];
    xyz_world = point->pos();
    n_reproj =
        point->n_succeeded_reproj_ - point->n_failed_reproj_;
  }
  else
  {
    xyz_world = ref_frame->T_world_cam() * ref_frame->getSeedPosInFrame(ref_index);
  }

  Keypoint px;
  if(!projectPointAndCheckVisibility(cur_frame, xyz_world, &px))
    return false;

  candidate = Reprojector::Candidate(
        ref_frame, ref_index, px, n_reproj, ref_frame->score_vec_[ref_index],
        ref_frame->type_vec_[ref_index]);

  return true;
}

bool projectPointAndCheckVisibility(
    const FramePtr& frame,
    const Eigen::Vector3d& xyz,
    Eigen::Vector2d* px)
{
  CHECK_NOTNULL(px);

  // transform point xyz to bearing vector f in camera frame
  const Eigen::Vector3d f = frame->T_f_w_ * xyz;

  // compute where the point projects and check visibility
  // TODO: insert reasonable clipping distance
  if(!frame->cam()->project3(f, px).isKeypointVisible())
    return false;
  const Eigen::Vector2i pxi = px->cast<int>();
  constexpr int kPatchSize = 8;
  if(!frame->cam()->isKeypointVisibleWithMargin(pxi, kPatchSize))
    return false;
  return true;
}

void setGridCellsOccupied(
    const Reprojector::Candidates& candidates,
    OccupandyGrid2D& grid)
{
  for(auto candidate : candidates)
  {
    size_t grid_index = grid.getCellIndex(candidate.cur_px.x(), candidate.cur_px.y(), 1);
    grid.setOccupied(grid_index);
  }
}

} // namespace reprojector_utils
} // namespace svo
