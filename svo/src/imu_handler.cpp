// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#include <svo/imu_handler.h>
#include <vikit/math_utils.h>
#include <vikit/csv_utils.h>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace svo {

PreintegratedImuMeasurement::PreintegratedImuMeasurement(
    const Eigen::Vector3d& omega_bias,
    const Eigen::Vector3d& acc_bias)
  : omega_bias_(omega_bias)
  , acc_bias_(acc_bias)
  , delta_t_ij_(Eigen::Vector3d::Zero())
  , delta_v_ij_(Eigen::Vector3d::Zero())
  , delta_R_ij_()
  , dt_sum_(0.0)
  , last_imu_measurement_set_(false)
{}

void PreintegratedImuMeasurement::addMeasurement(const ImuMeasurement &m)
{
  if(last_imu_measurement_set_)
  {
    const double dt = m.timestamp_-last_imu_measurement.timestamp_;
    const Eigen::Vector3d a = last_imu_measurement.linear_acceleration_-acc_bias_;
    const Eigen::Vector3d w = last_imu_measurement.angular_velocity_-omega_bias_;
    const Quaternion R_incr = Quaternion::exp(w*dt);

    // second order integration:
    delta_t_ij_ += delta_v_ij_*dt + (delta_R_ij_.rotate(a))*dt*dt*0.5;
    delta_v_ij_ += delta_R_ij_.rotate(a*dt);
    delta_R_ij_ = delta_R_ij_*R_incr;
    dt_sum_ += dt;
  }
  last_imu_measurement_set_ = true;
  last_imu_measurement = m;
}

void PreintegratedImuMeasurement::addMeasurements(const ImuMeasurements& ms)
{
  for(const ImuMeasurement& m : ms)
  {
    addMeasurement(m);
  }
}

ImuHandler::ImuHandler(
    const ImuCalibration& imu_calib,
    const ImuInitialization& imu_init)
  : imu_calib_(imu_calib)
  , imu_init_(imu_init)
  , acc_bias_(imu_init.acc_bias)
  , omega_bias_(imu_init.omega_bias)
{}

ImuHandler::~ImuHandler()
{}

bool ImuHandler::getMeasurements(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    const bool delete_old_measurements,
    ImuMeasurements& measurements)
{
  assert(new_cam_timestamp > old_cam_timestamp);
  ulock_t lock(measurements_mut_);
  if(measurements_.empty())
  {
    LOG(WARNING) << "don't have any imu measurements!";
    return false;
  }

  // Substract camera delay to get imu timestamp.
  const double t1 = old_cam_timestamp - imu_calib_.delay_imu_cam;
  const double t2 = new_cam_timestamp - imu_calib_.delay_imu_cam;

  // Find the right measurements for integration, note that the newest measurement
  // is at the front of the list!
  ImuMeasurements::iterator it1=measurements_.end(); // older timestamp
  ImuMeasurements::iterator it2=measurements_.end(); // newer timestamp
  bool it2_set = false;
  for(ImuMeasurements::iterator it=measurements_.begin(); it!=measurements_.end(); ++it)
  {
    if(!it2_set && it->timestamp_ < t2)
    {
      it2 = it;
      it2_set = true;
    }

    if(it->timestamp_ <= t1)
    {
      it1 = it;
      break;
    }
  }

  // check
  if(it1 == measurements_.end())
  {
    LOG(WARNING) << "need an older measurement for t1!";
    return false;
  }

  if(it2 == measurements_.end())
  {
    LOG(WARNING) << "need an older measurement for t2!";
    return false;
  }

  if(it1 == it2)
  {
    LOG(WARNING) << "not enough imu measurements!";
    return false;
  }

  if(t2-it2->timestamp_ > imu_calib_.max_imu_delta_t)
  {
    LOG(WARNING) << "newest imu measurement is too old for the image "
                    << t2-it2->timestamp_;
    return false;
  }

  // copy affected measurements
  ++it1; // make sure to copy the last element!
  measurements.insert(measurements.begin(), it2, it1);

  // change timestamp of oldest measurement
  measurements.back().timestamp_ = t1;

  // delete measurements that will not be used anymore
  if(delete_old_measurements)
    measurements_.erase(it1, measurements_.end());

  return true;
}

bool ImuHandler::getClosestMeasurement(
    const double timestamp,
    ImuMeasurement& measurement) const
{
  ulock_t lock(measurements_mut_);
  if(measurements_.empty())
  {
    LOG(WARNING) << "ImuHandler: don't have any imu measurements!";
    return false;
  }

  double dt_best = std::numeric_limits<double>::max();
  double img_timestamp_corrected = timestamp - imu_calib_.delay_imu_cam;
  for(const ImuMeasurement& m : measurements_)
  {
    const double dt = std::abs(m.timestamp_- img_timestamp_corrected);
    if(dt < dt_best)
    {
      dt_best = dt;
      measurement = m;
    }
  }

  if(dt_best > imu_calib_.max_imu_delta_t)
  {
    LOG(WARNING) << "ImuHandler: getClosestMeasurement: no measurement found!"
                    " closest measurement: " << dt_best*1000.0 << "ms.";
    return false;
  }
  return true;
}

bool ImuHandler::getRelativeRotationPrior(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    bool delete_old_measurements,
    Quaternion& R_oldimu_newimu)
{
  ImuMeasurements measurements;
  if(!getMeasurements(
       old_cam_timestamp, new_cam_timestamp, delete_old_measurements, measurements))
    return false;

  // integrate all measurements from t1 to t2
  R_oldimu_newimu.setIdentity();
  ImuMeasurements::reverse_iterator it = measurements.rbegin();
  ImuMeasurements::reverse_iterator it_plus = measurements.rbegin(); ++it_plus;
  for(; it != measurements.rend(); ++it, ++it_plus)
  {
    double dt = 0.0;
    if(it_plus == measurements.rend()) // only for newest measurement
      dt = new_cam_timestamp - imu_calib_.delay_imu_cam - it->timestamp_;
    else
      dt = it_plus->timestamp_ - it->timestamp_;

    const Eigen::Vector3d omega_corrected = it->angular_velocity_ - omega_bias_;
    R_oldimu_newimu = R_oldimu_newimu * Quaternion::exp(omega_corrected*dt);
  }
  return true;
}

bool ImuHandler::addImuMeasurement(
      const ImuMeasurement& m)
{
  ulock_t lock(measurements_mut_);
  measurements_.push_front(m); // new measurement is at the front of the list!
  return true;
}

bool ImuHandler::loadImuMeasurementsFromFile(const std::string& filename)
{
  ulock_t lock(measurements_mut_);
  std::ifstream fs(filename.c_str());
  if(!fs.is_open())
  {
    LOG(WARNING) << "Could not open imu file: " << filename;
    return false;
  }

  // add all imu messages to imu-handler
  size_t n = 0;
  while(fs.good() && !fs.eof())
  {
    if(fs.peek() == '#') // skip comments
      fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    size_t imu_id;
    double stamp, wx, wy, wz, ax, ay, az;
    fs >> imu_id >> stamp >> wx >> wy >> wz >> ax >> ay >> az;

//    // DEBUG **************************
//    if(imu_id % 10 != 0)
//      continue;
//    // ********************************

    const Eigen::Vector3d omega(wx, wy, wz);
    const Eigen::Vector3d linacc(ax, ay, az);
    const ImuMeasurement m(stamp, omega, linacc);
    measurements_.push_front(m);
    ++n;
  }
  VLOG(2) << "ImuHandler: Loaded " << n << " measurements.";
  return true;
}

bool ImuHandler::loadImuMeasurementsFromCsvFile(const std::string& filename)
{
  ulock_t lock(measurements_mut_);

  // open file
  std::ifstream csv_file_stream(filename, std::ios::in);
  CHECK(csv_file_stream.is_open()) << "Failed to open file " << filename << ".";
  CHECK(!csv_file_stream.eof()) << "File empty: " << filename;

  // skip header
  std::string header_line;
  std::getline(csv_file_stream, header_line);

  // read measurements
  std::vector<std::string> fields;
  size_t n = 0;
  while(vk::readNextLine(7, csv_file_stream, &fields))
  {
    int64_t stamp_ns = vk::convertStringToLongLong(fields[0]);
    Eigen::Vector3d omega = vk::convertStringsToEigenVector3d(fields[1], fields[2], fields[3]);
    Eigen::Vector3d linacc = vk::convertStringsToEigenVector3d(fields[4], fields[5], fields[6]);
    const ImuMeasurement m(
          static_cast<double>(stamp_ns)/1e9, omega, linacc); // TODO(cfo): save uint64_t timestamps!!!
    measurements_.push_front(m);
    ++n;
  }

  VLOG(2) << "ImuHandler: Loaded " << n << " measurements.";
  return true;
}

ImuCalibration ImuHandler::loadCalibrationFromFile(const std::string& filename)
{
  YAML::Node data = YAML::LoadFile(filename);
  ImuCalibration calib;
  if(data["imu_params"].IsDefined())
  {
    calib.delay_imu_cam = data["imu_params"]["delay_imu_cam"].as<double>();
    calib.max_imu_delta_t = data["imu_params"]["max_imu_delta_t"].as<double>();
    calib.saturation_accel_max = data["imu_params"]["acc_max"].as<double>();
    calib.saturation_omega_max = data["imu_params"]["omega_max"].as<double>();
    calib.gyro_noise_density = data["imu_params"]["sigma_omega_c"].as<double>();
    calib.acc_noise_density = data["imu_params"]["sigma_acc_c"].as<double>();
    calib.gyro_bias_random_walk_sigma = data["imu_params"]["sigma_omega_bias_c"].as<double>();
    calib.acc_bias_random_walk_sigma = data["imu_params"]["sigma_acc_bias_c"].as<double>();
    calib.gravity_magnitude = data["imu_params"]["g"].as<double>();
    calib.imu_rate = data["imu_params"]["imu_rate"].as<double>();
  }
  else
  {
    LOG(FATAL) << "Could not load IMU calibration from file";
  }
  return calib;
}

ImuInitialization ImuHandler::loadInitializationFromFile(const std::string& filename)
{
  YAML::Node data = YAML::LoadFile(filename);
  ImuInitialization init;
  if(data["imu_initialization"].IsDefined())
  {
    init.velocity = Eigen::Vector3d(
          data["imu_initialization"]["velocity"][0].as<double>(),
          data["imu_initialization"]["velocity"][1].as<double>(),
          data["imu_initialization"]["velocity"][2].as<double>());
    init.omega_bias = Eigen::Vector3d(
          data["imu_initialization"]["omega_bias"][0].as<double>(),
          data["imu_initialization"]["omega_bias"][1].as<double>(),
          data["imu_initialization"]["omega_bias"][2].as<double>());
    init.acc_bias = Eigen::Vector3d(
          data["imu_initialization"]["acc_bias"][0].as<double>(),
          data["imu_initialization"]["acc_bias"][1].as<double>(),
          data["imu_initialization"]["acc_bias"][2].as<double>());
    init.velocity_sigma = data["imu_initialization"]["velocity_sigma"].as<double>();
    init.omega_bias_sigma = data["imu_initialization"]["omega_bias_sigma"].as<double>();
    init.acc_bias_sigma = data["imu_initialization"]["acc_bias_sigma"].as<double>();
  }
  else
  {
    LOG(FATAL) << "Could not load IMU initialization from file";
  }
  return init;
}

bool ImuHandler::getAngularVelocity(
    double timestamp,
    Eigen::Vector3d& omega) const
{
  timestamp -= imu_calib_.delay_imu_cam; // correct for camera delay
  double min_delta = imu_calib_.max_imu_delta_t;
  for(ImuMeasurements::const_iterator it=measurements_.begin(); it!=measurements_.end(); ++it)
  {
    const double dt = std::fabs(it->timestamp_ - timestamp);
    if(dt < min_delta)
    {
      omega = it->angular_velocity_;
      min_delta = dt;
    }
  }
  if(min_delta < imu_calib_.max_imu_delta_t)
    return true;
  return false;
}

bool ImuHandler::getInitialAttitude(
    double timestamp,
    Quaternion& R_imu_world) const
{
  ImuMeasurement m;
  if(!getClosestMeasurement(timestamp, m))
  {
    LOG(WARNING) << "ImuHandler: Could not get initial attitude. No measurements!";
    return false;
  }

  // Set initial coordinate frame based on gravity direction.
  const Eigen::Vector3d& g = m.linear_acceleration_;
  const Eigen::Vector3d z = g.normalized(); // imu measures positive-z when static
  // TODO: make sure z != -1,0,0
  Eigen::Vector3d p(1,0,0);
  Eigen::Vector3d p_alternative(0,1,0);
  if(std::fabs(z.dot(p)) > std::fabs(z.dot(p_alternative)))
    p = p_alternative;
  Eigen::Vector3d y = z.cross(p); // make sure gravity is not in x direction
  y.normalize();
  const Eigen::Vector3d x = y.cross(z);
  Eigen::Matrix3d C_imu_world; // world unit vectors in imu coordinates
  C_imu_world.col(0) = x;
  C_imu_world.col(1) = y;
  C_imu_world.col(2) = z;

  VLOG(3) << "Initial Rotation = " << C_imu_world;

  R_imu_world = Quaternion(C_imu_world);
  return true;
}

void ImuHandler::reset()
{
  ulock_t lock(measurements_mut_);
  measurements_.clear();
}

} // namespace svo

