
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/common/imu_calibration.h>
#include <svo/backend/graph_manager.h>
#include <vikit/math_utils.h>

// boost
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// gtsam
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/linearExceptions.h>

namespace svo {

GraphManager::GraphManager(const GraphManagerOptions& options)
  : options_(options)
{}

GraphManager::~GraphManager()
{}

void GraphManager::initialize()
{
  // camera
  cam_calib_.reset(new gtsam::Cal3_S2(1.0, 1.0, 0.0, 0.0, 0.0));

  // reprojection error noise models
  const size_t kMaxPyrLevels = 6;
  if(options_.use_robust_px_noise)
  {
    const double kRobustKurtosis = 3.0;
    px_noise_pyr_.resize(kMaxPyrLevels);
    for(size_t level=0u; level<kMaxPyrLevels; ++level)
    {
      const double px_noise = options_.reproj_error_noise * (1<<level);
      px_noise_pyr_[level] =
          gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(kRobustKurtosis),
            gtsam::noiseModel::Isotropic::Sigma(2, px_noise));
    }
  }
  else
  {
    px_noise_pyr_.resize(kMaxPyrLevels);
    for(size_t level=0u; level<kMaxPyrLevels; ++level)
    {
      const double px_noise = options_.reproj_error_noise; // (1<<level);
      px_noise_pyr_[level] = gtsam::noiseModel::Isotropic::Sigma(2, px_noise);
    }
  }
  smart_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, options_.reproj_error_noise);
  smart_factor_params_.reset(new gtsam::SmartProjectionParams(
                               gtsam::HESSIAN,
                               gtsam::IGNORE_DEGENERACY, // creates rotation-only constraint
                               true, true));

  // TODO: retriangulation threshold is a const variable and cant be set in constructor.
  smart_factor_params_->setRankTolerance(1e-3);
//  smart_factor_params_->setEnableEPI(false); // nonlinear refinement after triangulation
//  smart_factor_params_->setLandmarkDistanceThreshold(100);
  smart_factor_params_->setDynamicOutlierRejectionThreshold(options_.smart_reproj_threshold);

  // position prior
  pose_prior_covariance_.setZero();
  pose_prior_covariance_.diagonal()[0] = options_.init_roll_pitch_sigma * options_.init_roll_pitch_sigma;
  pose_prior_covariance_.diagonal()[1] = options_.init_roll_pitch_sigma * options_.init_roll_pitch_sigma;
  pose_prior_covariance_.diagonal()[2] = options_.init_yaw_sigma * options_.init_yaw_sigma;
  pose_prior_covariance_.diagonal()[3] = options_.init_pos_sigma * options_.init_pos_sigma;
  pose_prior_covariance_.diagonal()[4] = options_.init_pos_sigma * options_.init_pos_sigma;
  pose_prior_covariance_.diagonal()[5] = options_.init_pos_sigma * options_.init_pos_sigma;
}

void GraphManager::initializeImuNoiseModels(
      const ImuCalibration& imu_calib,
      const ImuInitialization& imu_init)
{
  // prior bias
  Eigen::Matrix<double, 6, 1> prior_bias_sigmas;
  prior_bias_sigmas.head<3>().setConstant(imu_init.acc_bias_sigma);
  prior_bias_sigmas.tail<3>().setConstant(imu_init.omega_bias_sigma);
  imu_bias_prior_noise_ =
      gtsam::noiseModel::Diagonal::Sigmas(prior_bias_sigmas);

  // velocity prior
  velocity_prior_noise_ =
      gtsam::noiseModel::Isotropic::Sigma(3, imu_init.velocity_sigma);

  // Create the constant zero velocity noise model.
  static const double kZeroVelocitySigma = 1.0e-3;  // [m/s]
  zero_velocity_prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, kZeroVelocitySigma);

  // Calculate the measurement and integration noise covariances.
  // Note that the covariances are specified in continous-time.
  preintegration_params_ = PreintegratedImuMeasurements::Params::MakeSharedD(
      -imu_calib.gravity_magnitude);
  preintegration_params_->gyroscopeCovariance =
      std::pow(imu_calib.gyro_noise_density, 2.0) * Eigen::Matrix3d::Identity();
  preintegration_params_->accelerometerCovariance =
      std::pow(imu_calib.acc_noise_density, 2.0) * Eigen::Matrix3d::Identity();
  preintegration_params_->integrationCovariance =
      std::pow(imu_calib.imu_integration_sigma, 2.0) * Eigen::Matrix3d::Identity();
  preintegration_params_->biasAccCovariance =
      std::pow(imu_calib.acc_bias_random_walk_sigma, 2.0) * Eigen::Matrix3d::Identity();
  preintegration_params_->biasOmegaCovariance =
      std::pow(imu_calib.gyro_bias_random_walk_sigma, 2.0) * Eigen::Matrix3d::Identity();
  preintegration_params_->biasAccOmegaInt.setZero();
  preintegration_params_->omegaCoriolis = imu_calib.omega_coriolis;
  preintegration_params_->use2ndOrderCoriolis = false;
}

void GraphManager::reset()
{
  VLOG(3) << "Backend: Graph Manager reset";
  new_factors_.resize(0);
  new_values_.clear();
  new_smart_factors_.clear();
  smart_factors_map_.clear();
  last_added_state_index_ = -1;
}

// -----------------------------------------------------------------------------
void GraphManager::addLandmark(svo::Point& point)
{
  CHECK(!point.in_ba_graph_);

  // Add the initial landmark position expressed in the world frame.
  new_values_.insert(gtsam::Symbol('l', point.id()), gtsam::Point3(point.pos()));

  // add observations to smart-factor
  for(const KeypointIdentifier& obs : point.obs_)
  {
    if(const FramePtr frame = obs.frame.lock())
    {
      CHECK_GE(frame->in_ba_graph_vec_.size(), obs.keypoint_index_);
      CHECK(frame->landmark_vec_[obs.keypoint_index_]->id() == point.id()) << "Feature observation inconsistent!";
      CHECK(!frame->in_ba_graph_vec_[obs.keypoint_index_]) << "Feature is already in ba graph!";
      const Eigen::Vector2d uv = vk::project2(frame->f_vec_.col(obs.keypoint_index_));
      const size_t level = frame->level_vec_(obs.keypoint_index_);
      new_factors_.push_back(
            boost::make_shared<ProjectionFactor>(
              uv, px_noise_pyr_.at(level),
              gtsam::Symbol('x', frame->bundleId()),
              gtsam::Symbol('l', point.id()),
              cam_calib_,
              gtsam::Pose3(frame->T_imu_cam().getTransformationMatrix())));
      frame->in_ba_graph_vec_[obs.keypoint_index_] = true;
      VLOG(100) << "Reproj. error = "
                << dynamic_cast<ProjectionFactor*>(new_factors_.back().get())->unwhitenedError(new_values_).norm();

    }
  }

  VLOG(40) << "Added new smart landmark for point " << point.id();
}

void GraphManager::addObservationToLandmark(
    svo::Frame& frame,
    const size_t keypoint_index)
{
  CHECK(!frame.in_ba_graph_vec_[keypoint_index]);
  const svo::Point& point = *CHECK_NOTNULL(frame.landmark_vec_.at(keypoint_index).get());
  CHECK(point.in_ba_graph_);

  const Eigen::Vector2d uv = vk::project2(frame.f_vec_.col(keypoint_index));
  const size_t level = frame.level_vec_(keypoint_index);
  new_factors_.push_back(
        boost::make_shared<ProjectionFactor>(
          uv, px_noise_pyr_.at(level),
          gtsam::Symbol('x', frame.bundleId()),
          gtsam::Symbol('l', point.id()),
          cam_calib_,
          gtsam::Pose3(frame.T_imu_cam().getTransformationMatrix())));
  frame.in_ba_graph_vec_[keypoint_index] = true;

  VLOG(40) << "Added observation to point " << point.id();
}

// -----------------------------------------------------------------------------
void GraphManager::addSmartLandmark(svo::Point& point)
{
  CHECK(!point.in_ba_graph_);

  // TODO: We need to modify the Smart factor to take a different Camera-Body
  // transformation for every observation to generalize to camera-rig.
  gtsam::Pose3 T_imu_cam(point.getSeedFrame()->T_imu_cam().getTransformationMatrix());
  SmartFactor::shared_ptr new_factor(
        new SmartFactor(smart_noise_, cam_calib_, T_imu_cam, *smart_factor_params_));

  // add observations to smart-factor
  for(const KeypointIdentifier& obs : point.obs_)
  {
    if(const FramePtr frame = obs.frame.lock())
    {
      CHECK(!frame->in_ba_graph_vec_[obs.keypoint_index_]) << "Point is already in ba graph!";
      const Eigen::Vector2d uv = vk::project2(frame->f_vec_.col(obs.keypoint_index_));
      new_factor->add(uv, frame->bundleId());
      frame->in_ba_graph_vec_[obs.keypoint_index_] = true;
    }
  }

  // add new factor: Todo, add to optimization problem.
  new_smart_factors_.insert(std::make_pair(point.id(), new_factor));
  smart_factors_map_.insert(std::make_pair(point.id(), std::make_pair(new_factor, -1)));
  point.in_ba_graph_ = true;

  VLOG(40) << "Added new smart landmark for point " << point.id();
}

void GraphManager::addObservationToSmartLandmark(
    svo::Frame& frame,
    const size_t keypoint_index)
{
  CHECK(!frame.in_ba_graph_vec_[keypoint_index]);
  const svo::Point& point = *CHECK_NOTNULL(frame.landmark_vec_.at(keypoint_index).get());
  CHECK(point.in_ba_graph_);

  // Update existing smart-factor.
  auto smart_factors_map_it = smart_factors_map_.find(point.id());
  CHECK(smart_factors_map_it != smart_factors_map_.end())
      << "Tried to add a new observation to an existing landmark that is not in graph";

  const Eigen::Vector2d uv = vk::project2(frame.f_vec_.col(keypoint_index));

  SmartFactor::shared_ptr old_factor = smart_factors_map_it->second.first;
  SmartFactor::shared_ptr new_factor = boost::make_shared<SmartFactor>(*old_factor);
  new_factor->add(uv, gtsam::Symbol('x', frame.bundleId()));

  // update the factor
  if(smart_factors_map_it->second.second != -1)
  {
    // if slot is still -1, it means that the factor is not inserted yet in the graph
    new_smart_factors_.insert(std::make_pair(point.id(), new_factor));
  }
  smart_factors_map_it->second.first = new_factor;
  frame.in_ba_graph_vec_[keypoint_index] = true;

  VLOG(40) << "Added smart observation to point " << point.id();
}

// -----------------------------------------------------------------------------
void GraphManager::addCombinedImuFactor(
    const NFramesId& from_id,
    const NFramesId& to_id,
    const Eigen::Vector3d& acc_bias,
    const Eigen::Vector3d& gyr_bias,
    const double to_timestamp_sec,
    const ImuMeasurements& imu_measurements)
{
  // Create the preintegrated measurement.
  PreintegratedImuMeasurements pim(
        preintegration_params_,
        gtsam::imuBias::ConstantBias(acc_bias, gyr_bias));

  auto it = imu_measurements.rbegin();
  auto it_plus = imu_measurements.rbegin();
  for(++it_plus; it != imu_measurements.rend(); ++it, ++it_plus)
  {
    double dt = 0.0;
    if(it_plus == imu_measurements.rend()) // only for newest measurement
      dt = to_timestamp_sec - it->timestamp_;
    else
      dt = it_plus->timestamp_ - it->timestamp_;
    pim.integrateMeasurement(it->linear_acceleration_, it->angular_velocity_, dt);
  }

  new_factors_.push_back(
        boost::make_shared<gtsam::CombinedImuFactor>(
          gtsam::Symbol('x',from_id), gtsam::Symbol('v',from_id),
          gtsam::Symbol('x',to_id), gtsam::Symbol('v',to_id),
          gtsam::Symbol('b',from_id), gtsam::Symbol('b',to_id),
          pim));

  VLOG(40) << "Integrated " << imu_measurements.size() << " measurements"
           << " between frame " << from_id << " and " << to_id;
}

void GraphManager::addPosePriorFactor(
    const NFramesId& state_index,
    const Transformation& T_W_B)
{
  // Rotate initial pose uncertainty in world coordinates to body coordinates.
  Eigen::Matrix3d R_W_B = T_W_B.getRotationMatrix();
  Eigen::Matrix3d R_B_W = R_W_B.transpose();
  pose_prior_covariance_.topLeftCorner(3,3) =
      R_B_W * pose_prior_covariance_.topLeftCorner(3,3) * R_W_B;

  // Add pose prior.
  gtsam::SharedNoiseModel noise_init_pose =
      gtsam::noiseModel::Gaussian::Covariance(pose_prior_covariance_);
  new_factors_.push_back(
        boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >(
          gtsam::Symbol('x', state_index),
          gtsam::Pose3(T_W_B.getTransformationMatrix()),
          noise_init_pose));

  VLOG(40) << "Priors - Rotation Cov: \n" << pose_prior_covariance_.topLeftCorner(3,3);
  VLOG(40) << "Priors - Position Cov: \n" << pose_prior_covariance_.bottomRightCorner(3,3);

}

void GraphManager::addBiasPriorFactor(
    const NFramesId& state_index,
    const Eigen::Vector3d& acc_bias,
    const Eigen::Vector3d& gyr_bias)
{
  new_factors_.push_back(
        boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
          gtsam::Symbol('b', state_index),
          gtsam::imuBias::ConstantBias(acc_bias, gyr_bias),
          imu_bias_prior_noise_));

  VLOG(40) << "Priors - Bias Mean: Acc (" << acc_bias.transpose()
           << "), Gyro ( " << gyr_bias.transpose() << ")";
  VLOG(40) << "Priors - Bias Sigmas: " << imu_bias_prior_noise_->sigmas().transpose();
}

void GraphManager::addVelocityPriorFactor(
    const NFramesId& state_index,
    const Eigen::Vector3d& W_v)
{
  new_factors_.push_back(
        boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', state_index),
          W_v,
          velocity_prior_noise_));

  VLOG(40) << "Priors - Velocity";
}

void GraphManager::addZeroVelocityPriorFactor(
    const NFramesId& state_index)
{
  new_factors_.push_back(
        boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          gtsam::Symbol('v', state_index),
          Eigen::Vector3d::Zero(),
          zero_velocity_prior_noise_));
}

void GraphManager::augmentStateWithPose(
    const NFramesId& state_index,
    const Transformation& T_W_B)
{
  VLOG(4) << "Backend: Add new pose: " << state_index;
  new_values_.insert(
        gtsam::Symbol('x', state_index), gtsam::Pose3(T_W_B.getTransformationMatrix()));
  last_added_state_index_ = state_index;
}

void GraphManager::augmentStateWithVelocityAndBias(
    const NFramesId& state_index,
    const Eigen::Vector3d& W_v,
    const Eigen::Vector3d& acc_bias,
    const Eigen::Vector3d& gyr_bias)
{
  new_values_.insert(
        gtsam::Symbol('v', state_index), W_v);
  new_values_.insert(
        gtsam::Symbol('b', state_index), gtsam::imuBias::ConstantBias(acc_bias, gyr_bias));
}

void GraphManager::getUpdatesCopy(
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_states,
    std::vector<size_t>& delete_indices,
    std::vector<int>& smart_factor_point_ids)
{
  for(auto& s : new_smart_factors_)
  {
    new_factors.push_back(s.second);
    smart_factor_point_ids.push_back(s.first);
    const auto& it = smart_factors_map_.find(s.first);
    if(it->second.second != -1) // get current slot
      delete_indices.push_back(it->second.second);
  }
  new_factors.push_back(new_factors_.begin(), new_factors_.end());
  new_states = new_values_;
  new_smart_factors_.clear();
  new_factors_.resize(0);
  new_values_.clear();
}

void GraphManager::updateFactorSlots(
    const gtsam::FastVector<size_t>& new_slots,
    const std::vector<int>& smart_factor_point_ids)
{
  // update slots of new inserted indices:
  for(size_t i=0; i<smart_factor_point_ids.size(); ++i)
  {
    const auto& it = smart_factors_map_.find(smart_factor_point_ids.at(i));
    it->second.second = new_slots.at(i);
  }
}

} // namespace svo
