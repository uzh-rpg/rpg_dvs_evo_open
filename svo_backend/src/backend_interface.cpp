#include <svo/backend/backend_interface.h>

#include <svo/common/conversions.h>
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/map.h>
#include <svo/imu_handler.h>
#include <svo/pose_optimizer.h>
#include <svo/backend/graph_manager.h>
#include <svo/backend/backend_optimizer.h>

// gtsam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

namespace svo {

BackendInterface::BackendInterface(
    const BackendInterfaceOptions& options,
    const GraphManagerOptions& graph_manager_options,
    const OptimizerBackendOptions& optimizer_options)
  : options_(options)
{
  graph_.reset(new GraphManager(graph_manager_options));
  optimizer_.reset(new BackendOptimizer(optimizer_options, graph_));
}

void BackendInterface::reset()
{
  // TODO(cfo): Not threadsafe!
  VLOG(1) << "Backend: Reset";
  optimizer_->reset();
  graph_->reset();
  estimator_state_ = EstimatorState::kUninitialized;
  num_frames_in_backend_ = 0u;
}

void BackendInterface::initializeBackend()
{
  CHECK(!options_.add_imu_factors || (options_.add_imu_factors && imu_handler_))
      << "IMU Handler must be set when imu factors should be added to backend.";
  optimizer_->initialize();
  graph_->initialize();
  if(imu_handler_)
    graph_->initializeImuNoiseModels(imu_handler_->imu_calib_, imu_handler_->imu_init_);
}

void BackendInterface::bundleAdjustment(const FrameBundlePtr& frame_bundle)
{
  if(!frame_bundle->isKeyframe()) // TODO(cfo)
    return;

  switch (estimator_state_)
  {
    case EstimatorState::kUninitialized:
    {
      initializeBackend();
      {
        std::lock_guard<std::mutex> lock(graph_->mut_);
        ViNodeState state(frame_bundle->get_T_W_B());
        if(imu_handler_)
        {
          state.set_W_v_B(imu_handler_->imu_init_.velocity);
          state.setAccBias(imu_handler_->imu_init_.acc_bias);
          state.setGyroBias(imu_handler_->imu_init_.omega_bias);
        }
        addInitialStateAndPriorsToGraph(frame_bundle->getBundleId(), state);
        last_state_ = state;

      }
      estimator_state_ = EstimatorState::kInitialBundleAdjustment;
      VLOG(1) << "VIN: First frame set.";
      break;
    }

    case EstimatorState::kInitialBundleAdjustment:
    {
      {
        std::lock_guard<std::mutex> lock(graph_->mut_);
        graph_->augmentStateWithPose(frame_bundle->getBundleId(), frame_bundle->get_T_W_B());
        addVisualMeasurementsToGraph(frame_bundle);
        if(options_.add_imu_factors)
        {
          graph_->augmentStateWithVelocityAndBias(
                frame_bundle->getBundleId(), last_state_.get_W_v_B(),
                last_state_.getAccBias(), last_state_.getGyroBias());
          addInertialMeasurementsToGraph(frame_bundle);    
        }
        else if(num_frames_in_backend_ == 1u && !imu_handler_ &&
            frame_bundle->size() == 1u)
        {
          graph_->addPosePriorFactor(frame_bundle->getBundleId(),
                                     frame_bundle->get_T_W_B());

        }
      }

      if(num_frames_in_backend_ == options_.n_frames_in_init_ba - 1)
        estimator_state_ = EstimatorState::kRunning;

      VLOG(1) << "VIN: Estimator initializing: adding more frames."
              << ". Have already " << num_frames_in_backend_+1;
      break;
    }

    // Add frame to the problem with optimizing.
    case EstimatorState::kRunning:
    {
      {
        std::lock_guard<std::mutex> lock(graph_->mut_);
        graph_->augmentStateWithPose(frame_bundle->getBundleId(), frame_bundle->get_T_W_B());
        addVisualMeasurementsToGraph(frame_bundle);
        if(options_.add_imu_factors)
        {
          graph_->augmentStateWithVelocityAndBias(
                frame_bundle->getBundleId(), last_state_.get_W_v_B(),
                last_state_.getAccBias(), last_state_.getGyroBias());
          addInertialMeasurementsToGraph(frame_bundle);
        }
      }
      optimizer_->optimize();
      break;
    }

    default:
    {
      LOG(FATAL) << "Unhandled estimator state: " << static_cast<int>(estimator_state_);
      break;
    }
  }

  ++num_frames_in_backend_;
  last_added_index_ = frame_bundle->getBundleId();
  last_added_frame_stamp_ns_ = frame_bundle->getMinTimestampNanoseconds();
}

void BackendInterface::loadMapFromBundleAdjustment(
    const FrameBundlePtr& new_frames,
    const FrameBundlePtr& last_frames,
    const Map::Ptr& map)
{

  if(estimator_state_ != EstimatorState::kRunning)
  {
    VLOG(40) << "VIN: Backend not initialized yet.";
    return;
  }

  if(last_loaded_estimate_index_ == optimizer_->estimate_state_index_)
  {
    VLOG(3) << "VIN: This update was previously loaded.";
    return;
  }

  if(last_added_index_ != optimizer_->estimate_state_index_)
  {
    VLOG(3) << "VIN: Would like to update SVO map but Optimizer is not finished.";
    return;
  }

  // Try to get access to iSAM in order to retreive the new values and update
  // the SVO tracker map. However, iSAM might be busy optimizing. We wait a short
  // amount of time to to see if iSAM becomes available. Otherwise, we update the
  // map at the next iteration.
  std::lock_guard<std::mutex> estimate_lock(optimizer_->estimate_mut_);   // todo: estimate would be enough
  std::lock_guard<std::mutex> graph_lock(graph_->mut_); // to access smart_factors_map_. TODO shared_lock would be enough.
  const gtsam::Values& estimate = optimizer_->estimate_;

  // Update map.
  const FramePtr& last_kf = map->getLastKeyframe();
  Transformation T_kf_last = last_kf->T_imu_world()*last_frames->get_T_W_B();

  // 1. Update last state.
  {
    const gtsam::Pose3 pose =
        estimate.at<gtsam::Pose3>(gtsam::Symbol('x', last_added_index_));
    last_state_.set_T_W_B(Transformation(pose.matrix()));
    if(options_.add_imu_factors)
    {
      const Eigen::Vector3d& velocity =
          estimate.at<Eigen::Vector3d>(gtsam::Symbol('v', last_added_index_));
      last_state_.set_W_v_B(velocity);
      const gtsam::imuBias::ConstantBias& bias =
          estimate.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', last_added_index_));
      last_state_.setAccBias(bias.accelerometer());
      last_state_.setGyroBias(bias.gyroscope());
    }
  }

  // 2. update the poses of all keyframes.
  for(const auto& frame_pair : map->keyframes_) // TODO(cfo): we should store NFrames in map!
  {
    Frame& frame = *frame_pair.second;
    const Transformation T_W_B(
          estimate.at<gtsam::Pose3>(gtsam::Symbol('x', frame.bundleId())).matrix());
    frame.T_f_w_ = (T_W_B * frame.T_imu_cam()).inverse();
  }

  // 3. Update all 3d points.
  size_t n_reoptimized = 0;
  for(const auto& frame_pair : map->keyframes_) // TODO(cfo): we should store NFrames in map!
  {
    const Frame& frame = *frame_pair.second;
    for(size_t i = 0; i < frame.num_features_; ++i)
    {
      if(!frame.landmark_vec_[i])
        continue;

      Point& point = *frame.landmark_vec_[i];

      if(point.last_ba_update_ == optimizer_->estimate_state_index_)
        continue;

      if(point.in_ba_graph_)
      {
        if(options_.use_smart_factors)
        {
          // get smart-factor of 3d point:
          const auto& it = optimizer_->graph_->smart_factors_map_.find(point.id());
          CHECK(it != optimizer_->graph_->smart_factors_map_.end());
          if(it->second.first->isDegenerate())
          {
            map->addPointToTrash(frame.landmark_vec_[i]);
            VLOG(30) << "VIN: SmartFactor triangulation is degenerate!";
          }
          else
          {
            point.pos_ = it->second.first->point(estimate).get().vector();
          }
        }
        else
        {
          point.pos_ = estimate.at<gtsam::Point3>(gtsam::Symbol('l', point.id())).vector();
        }
      }
      else if(!point.in_ba_graph_ && point.nRefs() >= 2)
      {
        // point is not in optimization, therefore, we need to re-triangulate it.
        //point.triangulateLinear();
        point.optimize(5, false);
        ++n_reoptimized;
      }
      point.last_ba_update_ = optimizer_->estimate_state_index_;
    }

    //
    // TODO(cfo): update seed depth?
    //
  }

  // last frame is no keyframe, therefore it has not been affected by the above update.
  if(!last_frames->isKeyframe())
  {
    // TODO: if we have large scale change, this will not work! Also, update seeds.
    last_frames->set_T_W_B(last_kf->T_world_imu() * T_kf_last);
  }

  last_loaded_estimate_index_ = optimizer_->estimate_state_index_;
}

void BackendInterface::addVisualMeasurementsToGraph(
    const FrameBundlePtr& frame_bundle)
{
  // Statistics.
  size_t n_skipped_points_parallax = 0;
  size_t n_skipped_few_obs = 0;
  size_t n_features_already_in_graph = 0;
  size_t n_new_observations = 0;
  size_t n_new_landmarks = 0;

  for(size_t frame_id = 0; frame_id < frame_bundle->size(); ++frame_id)
  {
    Frame& frame = *frame_bundle->at(frame_id);
    for(size_t i = 0; i < frame.numFeatures(); ++i)
    {
      CHECK_GE(frame.num_features_, i);

      if(frame.landmark_vec_[i] == nullptr)
        continue;

      if(frame.in_ba_graph_vec_[i])
      {
        ++n_features_already_in_graph;
        continue;
      }

      Point& point = *frame.landmark_vec_.at(i);

      /*
      if(ignored_landmarks_.count(point.id()) > 0)
      {
        continue; // this landmark was discarted previously!
      }
      */

      if(point.in_ba_graph_)
      {
        graph_->addObservationToLandmark(frame, i);
        ++n_new_observations;
      }
      else
      {
        if(point.obs_.size() < options_.min_num_obs)
        {
          ++n_skipped_few_obs;
          continue;
        }

        if(std::isnan(point.pos_[0]))
        {
          LOG(ERROR) << "Point is nan!";
          continue;
        }

        if(point.getTriangulationParallax() < graph_->options_.min_parallax_thresh)
        {
          ++n_skipped_points_parallax;
          continue;
        }

        // TODO: We should first get all candidate points and sort them according
        //       to parallax angle and num observations. afterwards only add best
        //       N observations.
        graph_->addLandmark(point);
        point.in_ba_graph_ = true;

        ++n_new_landmarks;
      }
    } // landmarks
  } // frame-bundle

  VLOG(4) << "Backend: Added " << n_new_landmarks << " new landmarks";
  VLOG(4) << "Backend: Added " << n_new_observations << " new observations";
  VLOG(4) << "Backend: Observations already in graph: " << n_features_already_in_graph;
  VLOG(4) << "Backend: Adding points. Skipped because less than " << options_.min_num_obs << " observations: " << n_skipped_few_obs;
  VLOG(4) << "Backend: Adding points. Skipped because small parallax: " << n_skipped_points_parallax;
}

void BackendInterface::addInertialMeasurementsToGraph(
    const FrameBundlePtr& frame_bundle)
{
  ImuMeasurements imu_measurements;
  if (!imu_handler_->getMeasurements(
        last_added_frame_stamp_ns_ * common::conversions::kNanoSecondsToSeconds,
        frame_bundle->getMinTimestampNanoseconds() *
        common::conversions::kNanoSecondsToSeconds, true, imu_measurements))
  {
    LOG(WARNING) << "Could not retrieve IMU measurements.";
    return;
  }

  graph_->addCombinedImuFactor(
        last_added_index_,
        frame_bundle->getBundleId(),
        last_state_.getAccBias(),
        last_state_.getGyroBias(),
        frame_bundle->getMinTimestampNanoseconds() *
        common::conversions::kNanoSecondsToSeconds,
        imu_measurements);
  VLOG(4) << "Backend: Added " << imu_measurements.size() << " inertial measurements.";
}

void BackendInterface::addInitialStateAndPriorsToGraph(
    const BundleId& state_index,
    const ViNodeState& state)
{
  // Add initial estimate and visual-inertial priors to graph.
  graph_->augmentStateWithPose(state_index, state.get_T_W_B());
  graph_->addPosePriorFactor(state_index, state.get_T_W_B());
  if(options_.add_imu_factors)
  {
    graph_->augmentStateWithVelocityAndBias(
          state_index, state.get_W_v_B(), state.getAccBias(), state.getGyroBias());
    graph_->addBiasPriorFactor(
          state_index, state.getAccBias(), state.getGyroBias());
    graph_->addVelocityPriorFactor(
          state_index, state.get_W_v_B());
  }
}

void BackendInterface::startThread()
{
  optimizer_->startThread();
}

void BackendInterface::quitThread()
{
  optimizer_->quitThread();
}


} // namespace svo
