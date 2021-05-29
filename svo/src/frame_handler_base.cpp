// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#include <functional>
#include <future>
#include <memory>

#include <svo/common/conversions.h>
#include <svo/common/point.h>
#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo/direct/matcher.h>
#include <svo/frame_handler_base.h>
#include <svo/pose_optimizer.h>
#include <svo/tracker/feature_tracker.h>

#ifdef SVO_LOOP_CLOSING
#include <svo/online_loopclosing/loop_closing.h>
#endif

#ifdef SVO_WITH_CUDA
# include <svo/img_align/frame_gpu.h>
# include <svo/img_align/sparse_img_align_gpu.h>
#else
# include <svo/img_align/sparse_img_align.h>
#endif

#include "svo/abstract_bundle_adjustment.h"
#include "svo/initialization.h"
#include "svo/map.h"
#include "svo/reprojector.h"

namespace svo
{

// definition of global and static variables which were declared in the header
#ifdef SVO_TRACE
PerformanceMonitorPtr g_permon;
#endif

FrameHandlerBase::FrameHandlerBase(
    const BaseOptions& base_options,
    const ReprojectorOptions& reprojector_options,
    const DepthFilterOptions& depthfilter_options,
    const DetectorOptions& detector_options,
    const InitializationOptions& init_options,
    const FeatureTrackerOptions& tracker_options,
    const CameraBundle::Ptr& cameras)
  : options_(base_options)
  , cams_(cameras)
  , stage_(Stage::kPaused)
  , set_reset_(false)
  , set_start_(false)
  , map_(new Map)
  , acc_frame_timings_(10)
  , acc_num_obs_(10)
  , num_obs_last_(0)
  , tracking_quality_(TrackingQuality::kInsufficient)
  , relocalization_n_trials_(0)
{
  // sanity checks
  CHECK_EQ(reprojector_options.cell_size, detector_options.cell_size);

  need_new_kf_ = std::bind(&FrameHandlerBase::needNewKf, this, std::placeholders::_1);

#ifdef SVO_TRACE
  // Initialize Performance Monitor
  g_permon.reset(new vk::PerformanceMonitor());
  g_permon->addTimer("pyramid_creation");
  g_permon->addTimer("sparse_img_align");
  g_permon->addTimer("reproject");
  g_permon->addTimer("reproject_kfs");
  g_permon->addTimer("reproject_candidates");
  g_permon->addTimer("feature_align");
  g_permon->addTimer("pose_optimizer");
  g_permon->addTimer("point_optimizer");
  g_permon->addTimer("local_ba");
  g_permon->addTimer("tot_time");
  g_permon->addLog("timestamp");
  g_permon->addLog("img_align_n_tracked");
  g_permon->addLog("repr_n_mps");
  g_permon->addLog("repr_n_new_references");
  g_permon->addLog("sfba_thresh");
  g_permon->addLog("sfba_error_init");
  g_permon->addLog("sfba_error_final");
  g_permon->addLog("sfba_n_edges_final");
  g_permon->addLog("loba_n_erredges_init");
  g_permon->addLog("loba_n_erredges_fin");
  g_permon->addLog("loba_err_init");
  g_permon->addLog("loba_err_fin");
  g_permon->addLog("n_candidates");
  g_permon->addLog("dropout");
  g_permon->init("trace", options_.trace_dir);
#endif

  // init modules
  reprojectors_.reserve(cams_->getNumCameras());
  for(size_t camera_idx = 0; camera_idx < cams_->getNumCameras(); ++camera_idx)
  {
    reprojectors_.emplace_back(new Reprojector(reprojector_options, camera_idx));
  }
  SparseImgAlignOptions img_align_options;
  img_align_options.max_level = options_.img_align_max_level;
  img_align_options.min_level = options_.img_align_min_level;
  img_align_options.robustification = options_.img_align_robustification;
  img_align_options.use_distortion_jacobian =
      options_.img_align_use_distortion_jacobian;
  img_align_options.estimate_illumination_gain = options_.img_align_est_illumination_gain;
  img_align_options.estimate_illumination_offset = options_.img_align_est_illumination_offset;

#ifdef SVO_WITH_CUDA
  sparse_img_align_.reset(new SparseImgAlignGpu(
                            SparseImgAlignGpu::getDefaultSolverOptions(),
                            img_align_options));
#else
  sparse_img_align_.reset(new SparseImgAlign(
                            SparseImgAlign::getDefaultSolverOptions(),
                            img_align_options));
#endif // SVO_WITH_CUDA
  pose_optimizer_.reset(new PoseOptimizer(PoseOptimizer::getDefaultSolverOptions()));
  if(options_.poseoptim_using_unit_sphere)
    pose_optimizer_->setErrorType(PoseOptimizer::ErrorType::kBearingVectorDiff);

  // DEBUG ***
  //pose_optimizer_->initTracing(options_.trace_dir);
  DetectorOptions detector_options2 = detector_options;
  //detector_options2.detector_type = DetectorType::kGridGrad;

  depth_filter_.reset(new DepthFilter(depthfilter_options, detector_options2, cams_));
  initializer_ = initialization_utils::makeInitializer(
        init_options, tracker_options, detector_options, cams_);
  overlap_kfs_.resize(cams_->getNumCameras());

  VLOG(1) << "SVO initialized";
}

FrameHandlerBase::~FrameHandlerBase()
{
  VLOG(1) << "SVO destructor invoked";
}

//------------------------------------------------------------------------------
bool FrameHandlerBase::addImageBundle(
      const std::vector<cv::Mat>& imgs,
      const uint64_t timestamp)
{
  SVO_START_TIMER("pyramid_creation");
  CHECK_EQ(imgs.size(), cams_->getNumCameras());
  std::vector<FramePtr> frames;
  for(size_t i=0; i<imgs.size(); ++i)
  {
#ifdef SVO_WITH_CUDA
    frames.push_back(
          std::make_shared<FrameGpu>(cams_->getCameraShared(i), imgs[i].clone(), timestamp,
                                  options_.img_align_max_level+1));
#else
    frames.push_back(
          std::make_shared<Frame>(cams_->getCameraShared(i), imgs[i].clone(),
                                  timestamp, options_.img_align_max_level+1));
#endif // SVO_WITH_CUDA
    frames.back()->set_T_cam_imu(cams_->get_T_C_B(i));
    frames.back()->setNFrameIndex(i);
  }
  FrameBundlePtr frame_bundle(new FrameBundle(frames));
  SVO_STOP_TIMER("pyramid_creation");

  // Process frame bundle.
  return addFrameBundle(frame_bundle);
}

//------------------------------------------------------------------------------
bool FrameHandlerBase::addFrameBundle(const FrameBundlePtr& frame_bundle)
{
  VLOG(40) << "New Frame Bundle received: " << frame_bundle->getBundleId();
  CHECK_EQ(frame_bundle->size(), cams_->numCameras());

  // ---------------------------------------------------------------------------
  // Prepare processing.

  if(set_start_)
  {
    // Temporary copy rotation prior. TODO(cfo): fix this.
    Quaternion R_imu_world = R_imu_world_;
    bool have_rotation_prior = have_rotation_prior_;
    resetAll();
    R_imu_world_ = R_imu_world;
    have_rotation_prior_ = have_rotation_prior;
    setInitialPose(frame_bundle);
    stage_ = Stage::kInitializing;
  }

  if(stage_ == Stage::kPaused)
    return false;

  SVO_LOG("timestamp", frame_bundle->at(0)->getTimestampNSec());
  SVO_START_TIMER("tot_time");
  timer_.start();

  // ---------------------------------------------------------------------------
  // Add to pipeline.

  new_frames_ = frame_bundle;
  ++frame_counter_;

  // if we have bundle adjustment running in parallel thread, check if it has
  // computed a new map. in this case, we replace our map with the latest estimate
  if(bundle_adjustment_)
  {
    VLOG(40) << "Load frame from bundle adjustment.";
    bundle_adjustment_->loadMapFromBundleAdjustment(
          new_frames_, last_frames_, map_);
  }

  // Predict pose of new frame using motion prior.
  // TODO(cfo): remove same from processFrame in mono.
  if(last_frames_)
  {
    VLOG(40) << "Predict pose of new image using motion prior.";
    getMotionPrior(false);

    // set initial pose estimate
    for(size_t i = 0; i < new_frames_->size(); ++i)
    {
      new_frames_->at(i)->T_f_w_ =
          new_frames_->at(i)->T_cam_imu() * T_newimu_lastimu_prior_
          * last_frames_->at(i)->T_imu_world();
    }
  }

  // Perform tracking.
  UpdateResult res = processFrameBundle();

  if(res == UpdateResult::kKeyframe)
  {
    // Set flag in bundle. Before we only set each frame individually.
    new_frames_->setKeyframe();
  }

  if(bundle_adjustment_)
  {
    VLOG(40) << "Call bundle adjustment.";
    bundle_adjustment_->bundleAdjustment(new_frames_);
  }

  // ---------------------------------------------------------------------------
  // Finish pipeline.

  if(last_frames_)
  {
    // Set translation motion prior for next frame.
    t_lastimu_newimu_ =
        new_frames_->at(0)->T_imu_world().getRotation().rotate(
          new_frames_->at(0)->imuPos() - last_frames_->at(0)->imuPos());
  }

  // Statistics.
  acc_frame_timings_.push_back(timer_.stop());
  num_obs_last_ = new_frames_->numTrackedFeatures();
  if(stage_ == Stage::kTracking)
  {
    acc_num_obs_.push_back(num_obs_last_);
  }

  // Try relocalizing if tracking failed.
  if(res == UpdateResult::kFailure)
  {
    VLOG(2) << "Tracking failed: RELOCALIZE.";
    CHECK(stage_ == Stage::kTracking
          || stage_ == Stage::kInitializing
          || stage_ == Stage::kRelocalization);

    // Let's try to relocalize with respect to the last keyframe:
    reloc_keyframe_ = map_->getLastKeyframe();
    CHECK_NOTNULL(reloc_keyframe_.get());

    // Reset pose to previous frame to avoid crazy jumps.
    if(stage_ == Stage::kTracking && last_frames_)
    {
      for(size_t i = 0; i < last_frames_->size(); ++i)
        new_frames_->at(i)->T_f_w_ = last_frames_->at(i)->T_f_w_;
    }

    // Reset if we tried many times unsuccessfully to relocalize.
    if(stage_ == Stage::kRelocalization
       && relocalization_n_trials_ > options_.relocalization_max_trials)
    {
      VLOG(2) << "Relocalization failed " <<  options_.relocalization_max_trials
              << " times: RESET.";
      set_reset_ = true;
    }

    // Set stage.
    stage_ = Stage::kRelocalization;
    tracking_quality_ = TrackingQuality::kInsufficient;
  }

  // Set last frame.
  last_frames_ = new_frames_;
  new_frames_.reset();

#ifdef SVO_LOOP_CLOSING
  // get timestamp of the first svo frame (this is to be used in loop closure detection)
  if(last_frames_->getBundleId() == 1 && lc_ != nullptr)
  {
    lc_->first_frame_timestamp_ = last_frames_->at(0)->getTimestampNSec();
  }
  // add frame to loop closure detection database and run loop closure detection if required
  if(last_frames_->isKeyframe())
  {
    if(lc_ != nullptr)
    {
      lc_->addFrameToPR(last_frames_);
      if(lc_->publish_ == true)
      {
    	loop_closure_data.clear();
    	std::lock_guard < std::mutex > lock(lc_->mu_);
    	loop_closure_data = lc_->loop_closure_info_;
    	lc_->publish_ = false;
      }
    }
  }
#endif


  // Reset if we should.
  if(set_reset_)
  {
    resetAll();
  }

  // Reset rotation prior.
  have_rotation_prior_ = false;
  R_imulast_world_ = R_imu_world_;

  // Reset motion prior
  have_motion_prior_ = false;
  T_newimu_lastimu_prior_.setIdentity();

  // tracing
  SVO_LOG("dropout", static_cast<int>(res));
  SVO_STOP_TIMER("tot_time");
#ifdef SVO_TRACE
  g_permon->writeToFile();
#endif

  // Call callbacks.
  VLOG(40) << "Triggering addFrameBundle() callbacks...";
  triggerCallbacks(last_frames_);

  return true;
}

//------------------------------------------------------------------------------
void FrameHandlerBase::setRotationPrior(const Quaternion& R_imu_world)
{
  VLOG(40) << "Set rotation prior.";
  R_imu_world_ = R_imu_world;
  have_rotation_prior_ = true;
}

void FrameHandlerBase::setRotationIncrementPrior(const Quaternion& R_lastimu_newimu)
{
  VLOG(40) << "Set rotation increment prior.";
  R_imu_world_ = R_lastimu_newimu.inverse() * R_imulast_world_;
  have_rotation_prior_ = true;
}

//------------------------------------------------------------------------------
void FrameHandlerBase::setInitialPose(const FrameBundlePtr& frame_bundle) const
{
  if(have_rotation_prior_)
  {
    VLOG(40) << "Set initial pose: With rotation prior";
    for(size_t i = 0; i < frame_bundle->size(); ++i)
    {
      frame_bundle->at(i)->T_f_w_ =
          cams_->get_T_C_B(i) * Transformation(R_imu_world_, Vector3d::Zero());
    }
  }
  else if(frame_bundle->imu_measurements_.cols() > 0)
  {
    VLOG(40) << "Set initial pose: Use inertial measurements in frame to get gravity.";
    const Vector3d g = frame_bundle->imu_measurements_.topRows<3>().rowwise().sum();
    const Vector3d z = g.normalized(); // imu measures positive-z when static
    // TODO: make sure z != -1,0,0
    Vector3d p(1,0,0);
    Vector3d p_alternative(0,1,0);
    if(std::fabs(z.dot(p)) > std::fabs(z.dot(p_alternative)))
      p = p_alternative;
    Vector3d y = z.cross(p); // make sure gravity is not in x direction
    y.normalize();
    const Vector3d x = y.cross(z);
    Matrix3d C_imu_world; // world unit vectors in imu coordinates
    C_imu_world.col(0) = x;
    C_imu_world.col(1) = y;
    C_imu_world.col(2) = z;
    Transformation T_imu_world(Quaternion(C_imu_world), Eigen::Vector3d::Zero());
    frame_bundle->set_T_W_B(T_imu_world.inverse());
    VLOG(3) << "Initial Rotation = " << std::endl << C_imu_world.transpose() << std::endl;
  }
  else
  {
    VLOG(40) << "Set initial pose: set such that T_imu_world is identity.";
    for(size_t i = 0; i < frame_bundle->size(); ++i)
    {
      frame_bundle->at(i)->T_f_w_ = cams_->get_T_C_B(i) * T_world_imuinit.inverse();
    }
  }
}

//------------------------------------------------------------------------------
size_t FrameHandlerBase::sparseImageAlignment()
{
  // optimize the pose of the new frame such that it matches the pose of the previous frame best
  // this will improve the relative transformation between the previous and the new frame
  // the result is the number of feature points which could be tracked
  // this is a hierarchical KLT solver
  VLOG(40) << "Sparse image alignment.";
  SVO_START_TIMER("sparse_img_align");
  sparse_img_align_->reset();
  if(have_motion_prior_)
  {
    SVO_DEBUG_STREAM("Apply IMU Prior to Image align");
    double prior_trans = options_.img_align_prior_lambda_trans;
    if(map_->size() < 5)
      prior_trans = 0; // during the first few frames we don't want a prior (TODO)

    sparse_img_align_->setWeightedPrior(
          T_newimu_lastimu_prior_,
          0.0, 0.0,
          options_.img_align_prior_lambda_rot,
          prior_trans,
          0.0, 0.0);
  }
  sparse_img_align_->setMaxNumFeaturesToAlign(options_.img_align_max_num_features);
  size_t img_align_n_tracked = sparse_img_align_->run(last_frames_, new_frames_);

  SVO_STOP_TIMER("sparse_img_align");
  SVO_LOG("img_align_n_tracked", img_align_n_tracked);
  VLOG(40) << "Sparse image alignment tracked " << img_align_n_tracked << " features.";
  return img_align_n_tracked;
}

//------------------------------------------------------------------------------
size_t FrameHandlerBase::projectMapInFrame()
{
  VLOG(40) << "Project map in frame.";
  SVO_START_TIMER("reproject");

  // compute overlap keyframes
  for(size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
  {
    overlap_kfs_.at(camera_idx).clear();
    map_->getClosestNKeyframesWithOberlap(
          new_frames_->at(camera_idx),
          reprojectors_.at(camera_idx)->options_.max_n_kfs,
          &overlap_kfs_.at(camera_idx));
  }

  std::vector<std::vector<PointPtr>> trash_points;
  trash_points.resize(cams_->numCameras());
  if(options_.use_async_reprojectors && cams_->numCameras() > 1)
  {
    // start reprojection workers
    std::vector<std::future<void>> reprojector_workers;
    for(size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
    {
      auto func = std::bind(
            &Reprojector::reprojectFrames, reprojectors_.at(camera_idx).get(),
            new_frames_->at(camera_idx), overlap_kfs_.at(camera_idx),
            trash_points.at(camera_idx));
      reprojector_workers.push_back(std::async(std::launch::async, func));
    }

    // make sure all of them are finished
    for(size_t i=0; i < reprojector_workers.size(); ++i)
      reprojector_workers[i].get();
  }
  else
  {
    for(size_t camera_idx = 0; camera_idx < cams_->numCameras(); ++camera_idx)
    {
      reprojectors_.at(camera_idx)->reprojectFrames(
            new_frames_->at(camera_idx), overlap_kfs_.at(camera_idx),
            trash_points.at(camera_idx));
    }
  }

  // Effectively clear the points that were discarded by the reprojectors
  for(auto point_vec : trash_points)
    for(auto point : point_vec)
      map_->safeDeletePoint(point);

  // Count the total number of trials and matches for all reprojectors
  Reprojector::Statistics cumul_stats_;
  for(const ReprojectorPtr& reprojector : reprojectors_)
  {
    cumul_stats_.n_matches += reprojector->stats_.n_matches;
    cumul_stats_.n_trials += reprojector->stats_.n_trials;
  }

  SVO_STOP_TIMER("reproject");
  SVO_LOG("repr_n_new_references", cumul_stats_.n_matches);
  SVO_LOG("repr_n_mps", cumul_stats_.n_trials);
  VLOG(40) << "Reprojection:"
           << "\t nPoints = " << cumul_stats_.n_trials
           << "\t\t nMatches = " << cumul_stats_.n_matches;

  if(cumul_stats_.n_matches < options_.quality_min_fts)
    SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");

  return cumul_stats_.n_matches;
}

//------------------------------------------------------------------------------
size_t FrameHandlerBase::optimizePose()
{
  // pose optimization
  // optimize the pose of the frame in such a way, that the projection of all feature world coordinates
  // is not far off the position of the feature points within the frame. The optimization is done for all points
  // in the same time, hence optimizing frame pose.
  SVO_START_TIMER("pose_optimizer");
  pose_optimizer_->reset();
  if(have_motion_prior_)
  {
    VLOG(40) << "Apply prior to pose optimization";
    pose_optimizer_->setRotationPrior(
          new_frames_->get_T_W_B().getRotation().inverse(),
          options_.poseoptim_prior_lambda);
  }
  size_t sfba_n_edges_final = pose_optimizer_->run(new_frames_, options_.poseoptim_thresh);
  SVO_LOG("sfba_error_init", pose_optimizer_->stats_.reproj_error_before);
  SVO_LOG("sfba_error_final", pose_optimizer_->stats_.reproj_error_after);
  SVO_LOG("sfba_n_edges_final", sfba_n_edges_final);
  SVO_STOP_TIMER("pose_optimizer");
  SVO_DEBUG_STREAM("PoseOptimizer:"
                   << "\t ErrInit = " << pose_optimizer_->stats_.reproj_error_before
                   << "\t ErrFin = " << pose_optimizer_->stats_.reproj_error_after
                   << "\t nObs = " << sfba_n_edges_final);
  return sfba_n_edges_final;
}

//------------------------------------------------------------------------------
void FrameHandlerBase::optimizeStructure(
    const FrameBundle::Ptr& frames,
    int max_n_pts,
    int max_iter)
{
  VLOG(40) << "Optimize structure.";
  // some feature points will be optimized w.r.t keyframes they were observed
  // in the way that their projection error into all other keyframes is minimzed

  if(max_n_pts == 0)
    return; // don't return if max_n_pts == -1, this means we optimize ALL points

  SVO_START_TIMER("point_optimizer");
  for(const FramePtr& frame : frames->frames_)
  {
    bool optimize_on_sphere = false;
    if(frame->cam()->getType() == Camera::Type::kOmni)
      optimize_on_sphere = true;
    std::deque<PointPtr> pts;
    for(size_t i = 0; i < frame->num_features_; ++i)
    {
      if(frame->landmark_vec_[i] == nullptr || isEdgelet(frame->type_vec_[i]))
        continue;
      pts.push_back((frame->landmark_vec_[i]));
    }
    auto it_end = pts.end();
    if(max_n_pts > 0)
    {
      max_n_pts = std::min(static_cast<size_t>(max_n_pts), pts.size());
      std::nth_element(
            pts.begin(), pts.begin() + max_n_pts, pts.end(),
            [](const PointPtr& lhs, const PointPtr& rhs)
            {
              // we favour points that have not been optimized in a while
              return (lhs->last_structure_optim_ < rhs->last_structure_optim_);
            });
      it_end = pts.begin()+max_n_pts;
    }
    for(const PointPtr& point : pts)
    {
      point->optimize(max_iter, optimize_on_sphere);
      point->last_structure_optim_ = frame->id_;
    }
  }
  SVO_STOP_TIMER("point_optimizer");
}

//------------------------------------------------------------------------------
void FrameHandlerBase::upgradeSeedsToFeatures(const FramePtr& frame)
{
  VLOG(40) << "Upgrade seeds to features";
  size_t update_count = 0;
  for(size_t i = 0; i < frame->num_features_; ++i)
  {
    if(frame->landmark_vec_[i])
    {
      CHECK(frame->type_vec_[i] == FeatureType::kCorner ||
            frame->type_vec_[i] == FeatureType::kEdgelet);
      frame->landmark_vec_[i]->addObservation(frame, i);
    }
    else if(frame->seed_ref_vec_[i].keyframe)
    {
      SeedRef& ref = frame->seed_ref_vec_[i];

      // In multi-camera case, it might be that we already created a 3d-point
      // for this seed previously when processing another frame from the bundle.
      PointPtr point = ref.keyframe->landmark_vec_[ref.seed_id];
      if(point == nullptr)
      {
        // That's not the case. Therefore, create a new 3d point.
        Position xyz_world = ref.keyframe->T_world_cam()
            * ref.keyframe->getSeedPosInFrame(ref.seed_id);
        point = std::make_shared<Point>(xyz_world);
        ref.keyframe->landmark_vec_[ref.seed_id] = point;
        ref.keyframe->track_id_vec_[ref.seed_id] = point->id();
        point->addObservation(ref.keyframe, ref.seed_id);
      }

      // add reference to current frame.
      frame->landmark_vec_[i] = point;
      frame->track_id_vec_[i] = point->id();
      point->addObservation(frame, i);
      if(isCorner(ref.keyframe->type_vec_[ref.seed_id]))
      {
        ref.keyframe->type_vec_[ref.seed_id] = FeatureType::kCorner;
        frame->type_vec_[i] = FeatureType::kCorner;
      }
      else if(isEdgelet(ref.keyframe->type_vec_[ref.seed_id]))
      {
        ref.keyframe->type_vec_[ref.seed_id] = FeatureType::kEdgelet;
        frame->type_vec_[i] = FeatureType::kEdgelet;

        // Update the edgelet direction.
        double angle = feature_detection_utils::getAngleAtPixelUsingHistogram(frame->img_pyr_[frame->level_vec_[i]],
            (frame->px_vec_.col(i)/(1 << frame->level_vec_[i])).cast<int>(), 4u);
        frame->grad_vec_.col(i) = GradientVector(std::cos(angle), std::sin(angle));
      }
      else
      {
        CHECK(false) << "Seed-Type not known";
      }
      ++update_count;
    }

    // when using the feature-wrapper, we might copy some old references?
    frame->seed_ref_vec_[i].keyframe.reset();
    frame->seed_ref_vec_[i].seed_id = -1;
  }
  VLOG(40) << "NEW KEYFRAME: Updated " << update_count
           << " seeds to features in reference frame.";
}

//------------------------------------------------------------------------------
void FrameHandlerBase::resetCommon()
{
  stage_ = Stage::kPaused;
  tracking_quality_ = TrackingQuality::kInsufficient;
  set_reset_ = false;
  set_start_ = false;
  num_obs_last_ = 0;
  reloc_keyframe_.reset();
  relocalization_n_trials_ = 0;
  t_lastimu_newimu_ = Vector3d::Zero();
  have_motion_prior_ = false;
  T_newimu_lastimu_prior_.setIdentity();
  have_rotation_prior_ = false;
  for(auto& frame_vec : overlap_kfs_)
  {
    frame_vec.clear();
  }

  if(bundle_adjustment_)
  {
    bundle_adjustment_->reset();
  }

  new_frames_.reset();
  last_frames_.reset();
  map_->reset();

  sparse_img_align_->reset();
  depth_filter_->reset();
  initializer_->reset();

  VLOG(1) << "SVO RESET ALL";
}

//------------------------------------------------------------------------------
void FrameHandlerBase::setTrackingQuality(const size_t num_observations)
{
  tracking_quality_ = TrackingQuality::kGood;
  if(num_observations < options_.quality_min_fts)
  {
    SVO_WARN_STREAM_THROTTLE(
          0.5, "Tracking less than "<< options_.quality_min_fts <<" features!");
    tracking_quality_ = TrackingQuality::kInsufficient;
  }
  const int feature_drop =
      static_cast<int>(std::min(num_obs_last_, size_t(120))) - num_observations;
  if(feature_drop > options_.quality_max_fts_drop)
  {
    SVO_WARN_STREAM("Lost "<< feature_drop <<" features!");
    tracking_quality_ = TrackingQuality::kInsufficient;
  }
}

//------------------------------------------------------------------------------
bool FrameHandlerBase::needNewKf(const Transformation&)
{
  const std::vector<FramePtr>& visible_kfs = overlap_kfs_.at(0);
  if(options_.kfselect_criterion == KeyframeCriterion::DOWNLOOKING)
  {
    for(const auto& frame : visible_kfs)
    {
      // TODO: does not generalize to multiple cameras!
      Vector3d relpos = new_frames_->at(0)->T_cam_world()*frame->pos();
      if(fabs(relpos.x())/depth_median_ < options_.kfselect_min_dist &&
         fabs(relpos.y())/depth_median_ < options_.kfselect_min_dist*0.8 &&
         fabs(relpos.z())/depth_median_ < options_.kfselect_min_dist*1.3)
        return false;
    }
    VLOG(40) << "KF Select: NEW KEYFRAME";
    return true;
  }

  // else, FORWARD:
  size_t n_tracked_fts = new_frames_->numLandmarks();
  if(n_tracked_fts > options_.kfselect_numkfs_upper_thresh)
  {
    VLOG(40) << "KF Select: NO NEW KEYFRAME Above upper bound";
    return false;
  }

  // TODO: this only works for mono!
  if(last_frames_->at(0)->id() - map_->last_added_kf_id_ < options_.kfselect_min_num_frames_between_kfs)
  {
    VLOG(40) << "KF Select: NO NEW KEYFRAME We just had a KF";
    return false;
  }

  if(n_tracked_fts < options_.kfselect_numkfs_lower_thresh)
  {
    VLOG(40) << "KF Select: NEW KEYFRAME Below lower bound";
    return true;
  }

  // check that we have at least X disparity w.r.t to last keyframe
  if(options_.kfselect_min_disparity > 0)
  {
    int kf_id = map_->getLastKeyframe()->id();
    std::vector<double> disparities;
    const FramePtr& frame = new_frames_->at(0);
    disparities.reserve(frame->num_features_);
    for(size_t i = 0; i < frame->num_features_; ++i)
    {
      if(frame->landmark_vec_[i])
      {
        const Point::KeypointIdentifierList& observations = frame->landmark_vec_[i]->obs_;
        for(auto it = observations.rbegin(); it != observations.rend(); ++it)
        {
          if(it->frame_id == kf_id)
          {
            if(FramePtr kf = it->frame.lock())
              disparities.push_back(
                    (frame->px_vec_.col(i) - kf->px_vec_.col(it->keypoint_index_)).norm());
            break;
          }
        }
      }
      // TODO(cfo): loop also over seed references!
    }

    if(!disparities.empty())
    {
      double disparity = vk::getMedian(disparities);
      VLOG(40) << "KF Select: disparity = " << disparity;
      if(disparity < options_.kfselect_min_disparity)
      {
        VLOG(40) << "KF Select: NO NEW KEYFRAME disparity not large enough";
        return false;
      }
    }
  }

  for(const auto& kf : visible_kfs)
  {
    // TODO: doesn't generalize to rig!
    const double a =
        Quaternion::log(new_frames_->at(0)->T_f_w_.getRotation()
                        *kf->T_f_w_.getRotation().inverse()).norm()*180/M_PI;
    const double d = (new_frames_->at(0)->pos() - kf->pos()).norm();
    if(a < options_.kfselect_min_angle && d < options_.kfselect_min_dist_metric)
    {
      VLOG(40) << "KF Select: NO NEW KEYFRAME Min angle = " << a << ", min dist = " << d;
      return false;
    }
  }
  VLOG(40) << "KF Select: NEW KEYFRAME";
  return true;
}

//------------------------------------------------------------------------------
void FrameHandlerBase::getMotionPrior(const bool /*use_velocity_in_frame*/)
{
  if(have_rotation_prior_)
  {
    VLOG(40) << "Get motion prior from provided rotation prior.";
    T_newimu_lastimu_prior_ = Transformation(
          R_imulast_world_ * R_imu_world_.inverse(),t_lastimu_newimu_).inverse();
    have_motion_prior_ = true;
  }
  else if(new_frames_->imu_timestamps_ns_.cols() > 0)
  {
    VLOG(40) << "Get motion prior from integrated IMU measurements.";
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps_ns =
        new_frames_->imu_timestamps_ns_;
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements =
        new_frames_->imu_measurements_;
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero(); // TODO!
    const size_t num_measurements = imu_timestamps_ns.cols();
    Quaternion delta_R;
    for (size_t m_idx = 0u; m_idx < num_measurements - 1u; ++m_idx)
    {
      const double delta_t_seconds =
          (imu_timestamps_ns(m_idx + 1) - imu_timestamps_ns(m_idx)) *
          common::conversions::kNanoSecondsToSeconds;
      CHECK_LE(delta_t_seconds, 1e-12) <<
          "IMU timestamps need to be strictly increasing.";

      const Eigen::Vector3d w =
          imu_measurements.col(m_idx).tail<3>() - gyro_bias;
      const Quaternion R_incr = Quaternion::exp(w * delta_t_seconds);
      delta_R = delta_R * R_incr;
    }
    T_newimu_lastimu_prior_ =
        Transformation(delta_R, t_lastimu_newimu_).inverse();
    have_motion_prior_ = true;
  }
  /*
  else if(imu_handler_)
  {
    ImuMeasurements imu_measurements;
    if(!imu_handler_->getMeasurements(
         last_frames_->getTimestampSec(), new_frames_->getTimestampSec(), false, imu_measurements))
      return false;

    PreintegratedImuMeasurement preint(
          //      last_frames_->omega_bias_, last_frames_->omega_bias_); TODO: check why this worked before
          imuHandler()->getGyroscopeBias(), imuHandler()->getAccelerometerBias());
    preint.addMeasurements(imu_measurements);

    Vector3d t_last_new = t_lastimu_newimu_;
    if(use_velocity_in_frame)
    {
      t_last_new =
          preint.delta_t_ij_
          + last_frames_->T_f_w_.getRotation().rotate(
            last_frames_->velocity_*preint.dt_sum_
            - Vector3d(0, 0, -imu_handler_->imu_calib_.gravity_magnitude)*0.5*preint.dt_sum_*preint.dt_sum_);
    }

    Quaternion R_lastimu_newimu;
    imu_handler_->getRelativeRotationPrior(
          last_frames_->getTimestampSec(), new_frames_->getTimestampSec(), false, R_lastimu_newimu);
    T_newimu_lastimu_prior_ = Transformation(R_lastimu_newimu, t_last_new).inverse();
    return true;
  }
  */
  else if(options_.poseoptim_prior_lambda > 0 ||
          options_.img_align_prior_lambda_rot > 0 ||
          options_.img_align_prior_lambda_trans > 0)
  {
    VLOG(40) << "Get motion prior by assuming constant velocity.";
    T_newimu_lastimu_prior_ =
        Transformation(Quaternion(), t_lastimu_newimu_).inverse();
    have_motion_prior_ = true;
  }
  return;
}

//------------------------------------------------------------------------------
void FrameHandlerBase::setDetectorOccupiedCells(
    const size_t reprojector_grid_idx,
    const DetectorPtr& feature_detector)
{
  CHECK_EQ(feature_detector->grid_.size(), reprojectors_.at(reprojector_grid_idx)->grid_->size());
  feature_detector->grid_.occupancy_ =
      reprojectors_.at(reprojector_grid_idx)->grid_->occupancy_;
}

void FrameHandlerBase::setFirstFrames(const std::vector<FramePtr>& first_frames)
{
  resetAll();
  last_frames_.reset(new FrameBundle(first_frames));
  for(auto f : last_frames_->frames_)
  {
    f->setKeyframe();
    map_->addKeyframe(f);
  }
  stage_ = Stage::kTracking;
}

std::vector<FramePtr> FrameHandlerBase::closeKeyframes() const
{
  std::vector<FramePtr> close_kfs;
  for(const auto& kfs : overlap_kfs_)
  {
    close_kfs.insert(close_kfs.begin(), kfs.begin(), kfs.end());
  }
  return close_kfs;
}

void FrameHandlerBase::setCompensation(const bool do_compensation)
{
  sparse_img_align_->setCompensation(do_compensation);
  for (const ReprojectorPtr& rep : reprojectors_)
  {
    rep->options_.affine_est_gain = do_compensation;
  }
  depth_filter_->getMatcher().options_.affine_est_gain_ = do_compensation;
}

} // namespace svo
