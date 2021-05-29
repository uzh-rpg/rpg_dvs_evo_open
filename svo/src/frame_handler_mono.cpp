// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/img_align/sparse_img_align.h>
#include <svo/abstract_bundle_adjustment.h>
#include <svo/direct/depth_filter.h>
#include <svo/tracker/feature_tracking_types.h>
#include <svo/initialization.h>
#include <svo/direct/feature_detection.h>
#include <svo/reprojector.h>
#include <vikit/performance_monitor.h>

namespace svo {

FrameHandlerMono::FrameHandlerMono(
    const BaseOptions& base_options,
    const DepthFilterOptions& depth_filter_options,
    const DetectorOptions& feature_detector_options,
    const InitializationOptions& init_options,
    const ReprojectorOptions& reprojector_options,
    const FeatureTrackerOptions& tracker_options,
    const CameraBundle::Ptr& cam)
  : FrameHandlerBase(
      base_options, reprojector_options, depth_filter_options,
      feature_detector_options, init_options, tracker_options, cam)
{ ; }

UpdateResult FrameHandlerMono::processFrameBundle()
{
  UpdateResult res = UpdateResult::kFailure;
  if(stage_ == Stage::kTracking)
    res = processFrame();
  else if(stage_ == Stage::kInitializing)
    res = processFirstFrame();
  else if(stage_ == Stage::kRelocalization)
    res = relocalizeFrame(Transformation(), reloc_keyframe_);

  return res;
}

void FrameHandlerMono::addImage(
    const cv::Mat& img,
    const uint64_t timestamp)
{
  addImageBundle({img}, timestamp);
}

UpdateResult FrameHandlerMono::processFirstFrame()
{
  if(!initializer_->have_depth_prior_)
  {
    initializer_->setDepthPrior(options_.init_map_scale);
  }
  if(have_rotation_prior_)
  {
    VLOG(2) << "Setting absolute orientation prior";
    initializer_->setAbsoluteOrientationPrior(newFrame()->T_cam_imu().getRotation() * R_imu_world_);
  }
  const auto res = initializer_->addFrameBundle(new_frames_);

  if(res == InitResult::kTracking)
    return UpdateResult::kDefault;

  // make old frame keyframe
  initializer_->frames_ref_->setKeyframe();
  initializer_->frames_ref_->at(0)->setKeyframe();
  map_->addKeyframe(initializer_->frames_ref_->at(0));
  if(bundle_adjustment_)
  {
    bundle_adjustment_->bundleAdjustment(initializer_->frames_ref_);
  }

  // make new frame keyframe
  newFrame()->setKeyframe();
  frame_utils::getSceneDepth(newFrame(), depth_median_, depth_min_, depth_max_);
  VLOG(40) << "Current Frame Depth: " << "min: " << depth_min_
          << ", max: " << depth_max_ << ", median: " << depth_median_;
  depth_filter_->addKeyframe(
              newFrame(), depth_median_, 0.5*depth_min_, depth_median_*1.5);
  VLOG(40) << "Updating seeds in second frame using last frame...";
  depth_filter_->updateSeeds({ newFrame() }, lastFrameUnsafe());

  // add frame to map
  map_->addKeyframe(newFrame());
  stage_ = Stage::kTracking;
  initializer_->reset();
  VLOG(1) << "Init: Selected second frame, triangulated initial map.";
  return UpdateResult::kKeyframe;
}

UpdateResult FrameHandlerMono::processFrame()
{
  // ---------------------------------------------------------------------------
  // tracking

  // STEP 1: Sparse Image Align
  VLOG(40) << "===== Sparse Image Alignment =====";
  size_t n_tracked_features = 0;
  sparseImageAlignment();

  // STEP 2: Map Reprojection & Feature Align
  VLOG(40) << "===== Project Map to Current Frame =====";
  n_tracked_features = projectMapInFrame();
  if(n_tracked_features < options_.quality_min_fts)
    return UpdateResult::kFailure;

  // STEP 3: Pose & Structure Optimization
  VLOG(40) << "===== Pose Optimization =====";
  n_tracked_features = optimizePose();
  if(n_tracked_features < options_.quality_min_fts)
    return UpdateResult::kFailure;
  optimizeStructure(new_frames_, options_.structure_optimization_max_pts, 5);

  // return if tracking bad
  setTrackingQuality(n_tracked_features);
  if(tracking_quality_ == TrackingQuality::kInsufficient)
    return UpdateResult::kFailure;

  // ---------------------------------------------------------------------------
  // select keyframe
  VLOG(40) << "===== Keyframe Selection =====";
  frame_utils::getSceneDepth(newFrame(), depth_median_, depth_min_, depth_max_);
  VLOG(40) << "Current Frame Depth: " << "min: " << depth_min_
           << ", max: " << depth_max_ << ", median: " << depth_median_;
  if(!need_new_kf_(newFrame()->T_f_w_)
     || tracking_quality_ == TrackingQuality::kBad
     || stage_ == Stage::kRelocalization)
  {
    if(tracking_quality_ == TrackingQuality::kGood)
    {
      VLOG(40) << "Updating seeds in overlapping keyframes...";
      CHECK(!overlap_kfs_.empty());
      depth_filter_->updateSeeds(overlap_kfs_.at(0), newFrame());
    }
    return UpdateResult::kDefault;
  }
  newFrame()->setKeyframe();
  VLOG(40) << "New keyframe selected.";

  // ---------------------------------------------------------------------------
  // new keyframe selected
  upgradeSeedsToFeatures(newFrame());

  // init new depth-filters, set feature-detection grid-cells occupied that
  // already have a feature
  //
  // TODO: we should also project all seeds first! to make sure that we don't
  //       initialize seeds in the same location!!! this can be done in the
  //       depth-filter
  //
  {
    DepthFilter::ulock_t lock(depth_filter_->feature_detector_mut_);
    setDetectorOccupiedCells(0, depth_filter_->feature_detector_);
  } // release lock
  depth_filter_->addKeyframe(
        newFrame(), depth_median_, 0.5*depth_min_, depth_median_*1.5);

  if(options_.update_seeds_with_old_keyframes)
  {
    VLOG(40) << "Updating seeds in current frame using last frame...";
    depth_filter_->updateSeeds({ newFrame() }, lastFrameUnsafe());
    VLOG(40) << "Updating seeds in current frame using overlapping keyframes...";
    for(const FramePtr& old_keyframe : overlap_kfs_.at(0))
      depth_filter_->updateSeeds({ newFrame() }, old_keyframe);
  }

  VLOG(40) << "Updating seeds in overlapping keyframes...";
  depth_filter_->updateSeeds(overlap_kfs_.at(0), newFrame());

  // add keyframe to map
  map_->addKeyframe(newFrame());

  // if limited number of keyframes, remove the one furthest apart
  if(options_.max_n_kfs > 2 && map_->size() >= options_.max_n_kfs)
  {
    FramePtr furthest_frame = map_->getFurthestKeyframe(newFrame()->pos());
    map_->removeKeyframe(furthest_frame->id());
  }
  return UpdateResult::kKeyframe;
}

UpdateResult FrameHandlerMono::relocalizeFrame(
    const Transformation& /*T_cur_ref*/,
    const FramePtr& ref_keyframe)
{
  ++relocalization_n_trials_;
  if(ref_keyframe == nullptr)
    return UpdateResult::kFailure;

  VLOG_EVERY_N(1, 20) << "Relocalizing frame";
  FrameBundle::Ptr ref_frame(new FrameBundle({ref_keyframe}));
  last_frames_ = ref_frame;
  UpdateResult res = processFrame();
  if(res == UpdateResult::kDefault)
  {
    // Reset to default mode.
    stage_ = Stage::kTracking;
    relocalization_n_trials_ = 0;
    VLOG(1) << "Relocalization successful.";
  }
  else
  {
    // reset to last well localized pose
    newFrame()->T_f_w_ = ref_keyframe->T_f_w_;
  }
  return res;
}

void FrameHandlerMono::resetAll()
{
  resetCommon();
}

FramePtr FrameHandlerMono::lastFrame() const
{
  return (last_frames_ == nullptr) ? nullptr : last_frames_->at(0);
}

const FramePtr& FrameHandlerMono::newFrame() const
{
  return new_frames_->frames_[0];
}

const FramePtr& FrameHandlerMono::lastFrameUnsafe() const
{
  return last_frames_->frames_[0];
}

bool FrameHandlerMono::haveLastFrame() const
{
  return (last_frames_ != nullptr);
}

} // namespace svo
