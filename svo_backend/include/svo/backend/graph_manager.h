#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>

#include <svo/common/types.h>
#include <svo/common/transformation.h>
#include <svo/backend/smart_factors_fwd.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>

namespace svo {

// fwd
struct ImuCalibration;
struct ImuInitialization;
struct ImuMeasurement;
typedef std::deque<ImuMeasurement> ImuMeasurements; // TODO(cfo)
class Point;
class Frame;

struct GraphManagerOptions
{
  double reproj_error_noise = 1.0/370.0;
  double smart_reproj_threshold = 3.0/370.0; // set -1 to deactivate!
  double min_parallax_thresh = 5.0/180*M_PI;
  bool trace_tracks = false;
  bool use_robust_px_noise = false;
  double init_pos_sigma = 0.001;
  double init_roll_pitch_sigma = 0.03; // 2 / 180.0 * M_PI;
  double init_yaw_sigma = 0.78; // 45.0 / 180.0 * M_PI
};

class GraphManager
{
public:
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  using NFramesId = int;
  using Ptr = std::shared_ptr<GraphManager>;
  using PreintegratedImuMeasurements = gtsam::PreintegratedCombinedMeasurements;
  using CombinedPreintegratedMeasurementPtr = std::shared_ptr<PreintegratedImuMeasurements>;
  using PointIdSmartFactorMap = std::unordered_map<int, boost::shared_ptr<SmartFactor>>;

  GraphManager(const GraphManagerOptions& options);
  ~GraphManager();

  GraphManagerOptions options_;
  gtsam::Cal3_S2::shared_ptr cam_calib_;
  boost::shared_ptr<PreintegratedImuMeasurements::Params> preintegration_params_;
  std::vector<gtsam::SharedNoiseModel> px_noise_pyr_;
  gtsam::SharedNoiseModel smart_noise_;
  gtsam::SharedNoiseModel imu_bias_prior_noise_;
  gtsam::SharedNoiseModel velocity_prior_noise_;
  gtsam::SharedNoiseModel zero_velocity_prior_noise_;
  Eigen::Matrix<double, 6, 6> pose_prior_covariance_;
  std::unique_ptr<gtsam::SmartProjectionParams> smart_factor_params_;

  /// Updates to be considered for next optimization iteration. MUTEX PROTECTED
  /// @{

  std::mutex mut_;
  gtsam::NonlinearFactorGraph new_factors_; // new factors to be added
  gtsam::Values new_values_;                // new states to be added
  PointIdSmartFactorMap new_smart_factors_; // pointId -> {SmartFactorPtr}
  SmartFactorMap smart_factors_map_;        // pointId -> {SmartFactorPtr, SlotIndex}
  BundleId last_added_state_index_ = -1;

  /// @}

  void initialize();

  void initializeImuNoiseModels(
        const ImuCalibration& imu_calib,
        const ImuInitialization& imu_init);

  void reset();

  // Landmarks
  void addLandmark(svo::Point& point);

  void addObservationToLandmark(
      svo::Frame& frame,
      const size_t keypoint_index);

  void addSmartLandmark(svo::Point& point);

  void addObservationToSmartLandmark(
      svo::Frame& frame,
      const size_t keypoint_index);

  void addCombinedImuFactor(
      const NFramesId& from_id,
      const NFramesId& to_id,
      const Eigen::Vector3d& acc_bias,
      const Eigen::Vector3d& gyr_bias,
      const double to_timestamp_sec,
      const ImuMeasurements& imu_measurements);

  // Prior Factors
  void addPosePriorFactor(
      const NFramesId& state_index,
      const Transformation& T_W_B);

  void addBiasPriorFactor(
      const NFramesId& state_index,
      const Eigen::Vector3d& acc_bias,
      const Eigen::Vector3d& gyro_bias);

  void addVelocityPriorFactor(
      const NFramesId& state_index,
      const Eigen::Vector3d& W_v);

  void addZeroVelocityPriorFactor(
      const NFramesId& state_index);

  // Augment State
  void augmentStateWithPose(
      const NFramesId& state_index,
      const Transformation& T_W_B);

  void augmentStateWithVelocityAndBias(
      const NFramesId& state_index,
      const Eigen::Vector3d& W_v,
      const Eigen::Vector3d& acc_bias,
      const Eigen::Vector3d& gyr_bias);

  // Retrieval (Always lock mut_ when using)
  void getUpdatesCopy(
      gtsam::NonlinearFactorGraph& graph_updates,
      gtsam::Values& value_updates,
      std::vector<size_t>& delete_indices,
      std::vector<int>& smart_factor_point_ids);

  void updateFactorSlots(
      const gtsam::FastVector<size_t>& new_slots,
      const std::vector<int>& smart_factor_point_ids);

};

} // namespace svo
