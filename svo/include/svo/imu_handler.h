// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#pragma once

#include <memory>          // std::shared_ptr
#include <mutex>           // std::mutex
#include <iostream>
#include <fstream>
#include <deque>
#include <svo/common/types.h>
#include <svo/common/transformation.h>
#include <svo/common/imu_calibration.h>

namespace svo {

class PreintegratedImuMeasurement
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d omega_bias_;
  Eigen::Vector3d acc_bias_;
  Eigen::Vector3d delta_t_ij_;
  Eigen::Vector3d delta_v_ij_;
  Quaternion delta_R_ij_;
  double dt_sum_;

  PreintegratedImuMeasurement(
      const Eigen::Vector3d& omega_bias,
      const Eigen::Vector3d& acc_bias);

  /// Add single measurements to be integrated
  void addMeasurement(const ImuMeasurement& m);

  /// Add many measurements to be integrated
  void addMeasurements(const ImuMeasurements& ms);

private:
  bool last_imu_measurement_set_;
  ImuMeasurement last_imu_measurement;
};

class ImuHandler
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ImuHandler> Ptr;
  typedef std::mutex mutex_t;
  typedef std::unique_lock<mutex_t> ulock_t;

  ImuCalibration imu_calib_;
  ImuInitialization imu_init_;

  // TODO: make private
  mutable mutex_t bias_mut_;
  Eigen::Vector3d acc_bias_; //!< Accleration bias used during preintegration
  Eigen::Vector3d omega_bias_; //!< Angular rate bias values used during preintegration

  ImuHandler(
      const ImuCalibration& imu_calib,
      const ImuInitialization& imu_init);
  ~ImuHandler();


  const Eigen::Vector3d& getAccelerometerBias() const
  {
    ulock_t lock(bias_mut_);
    return acc_bias_;
  }

  const Eigen::Vector3d& getGyroscopeBias() const
  {
    ulock_t lock(bias_mut_);
    return omega_bias_;
  }

  void setAccelerometerBias(const Eigen::Vector3d& acc_bias)
  {
    ulock_t lock(bias_mut_);
    acc_bias_ = acc_bias;
  }

  void setGyroscopeBias(const Eigen::Vector3d& omega_bias)
  {
    ulock_t lock(bias_mut_);
    omega_bias_ = omega_bias;
  }

  ImuMeasurements getMeasurementsCopy() const
  {
    ulock_t lock(measurements_mut_);
    return measurements_;
  }

  /// Get IMU measurements in some time interval. Note that you have to provide
  /// the camera timestamps. Internally, given the calibration it corrects the
  /// timestamps for delays.
  bool getMeasurements(
      const double old_cam_timestamp, // seconds
      const double new_cam_timestamp, // seconds
      const bool delete_old_measurements,
      ImuMeasurements& measurements);

  bool getClosestMeasurement(
      const double timestamp,
      ImuMeasurement& measurement) const;

  // deprecated, use preintegrated imu measurement!
  /// Gets relative transformation in IMU coordinate frame
  bool getRelativeRotationPrior(
      const double old_cam_timestamp,
      const double new_cam_timestamp,
      bool delete_old_measurements,
      Quaternion& R_oldimu_newimu);

  bool getAngularVelocity(
      double timestamp,
      Eigen::Vector3d& omega) const;

  /// Assumes we are in hover condition and estimates the inital orientation by
  /// estimating the gravity direction. The yaw direction is not deterministic.
  bool getInitialAttitude(
      double timestamp,
      Quaternion& R_imu_world) const;

  bool addImuMeasurement(const ImuMeasurement& measurement);

  bool loadImuMeasurementsFromFile(const std::string& filename);

  bool loadImuMeasurementsFromCsvFile(const std::string& filename);

  static Eigen::Matrix3d integrateGyroMeasurement(
      const Eigen::Vector3d& omega_measured,
      const Eigen::Matrix3d& R_cam_imu,
      const double delta_t);

  static ImuCalibration loadCalibrationFromFile(const std::string& filename);
  static ImuInitialization loadInitializationFromFile(const std::string& filename);

  void reset();

private:

  mutable mutex_t measurements_mut_;
  ImuMeasurements measurements_; ///< Newest measurement is at the front of the list
  std::ofstream ofs_; //!< File stream for tracing the received measurments
};

} // namespace svo
