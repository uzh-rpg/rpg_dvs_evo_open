#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <vikit/params_helper.h>
#include <vikit/blender_utils.h>
#include <vikit/sample.h>
#include <svo/common/conversions.h>
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/imu_handler.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/direct/feature_detection.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo_ros/svo_interface.h>
#include <svo/test_utils/synthetic_dataset.h>
#include <svo/reprojector.h>
#include <svo_ros/visualizer.h>
#include <svo_ros/csv_dataset_reader.h>

#ifdef SVO_USE_BACKEND
#include <svo/backend/backend_interface.h>
#include <svo/backend/backend_optimizer.h>
#include <svo_ros/backend_visualizer.h>
#endif

namespace svo {

class BenchmarkNode : public SvoInterface
{
  int frame_count_ = 0;
  std::ofstream trace_gt_pose_;
  std::ofstream trace_est_pose_;
  std::ofstream trace_trans_error_;
  std::ofstream trace_rot_error_;
  std::ofstream trace_depth_error_;
  double img_noise_sigma_;
  size_t sleep_us_;

public:

  BenchmarkNode(
      const PipelineType& type,
      const ros::NodeHandle& nh,
      const ros::NodeHandle& pnh);

  void traceGroundtruth(const Transformation& T_w_gt, const double timestamp);
  void tracePose(const Transformation& T_w_f, const int64_t timestamp);
  void tracePoseError(const Transformation& T_f_gt, const double timestamp);
  void tracePoseKitti(const Transformation& T_w_f);
  void traceDepthError(const FramePtr& frame, const cv::Mat& depthmap);
  void addNoiseToImage(cv::Mat& img, double sigma);
  void runBenchmark(const std::string& dataset_dir);
  void runBlenderBenchmark(const std::string& dataset_dir, bool depthmap_has_zbuffer=false);
  void runKittiBenchmark(const std::string& dataset_dir);
  void runEurocBenchmark(const std::string& dataset_dir);
  void runArrayBenchmark(const std::string& dataset_dir);
};

BenchmarkNode::BenchmarkNode(
    const PipelineType& type,
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh)
  : SvoInterface(type, nh, pnh)
  , img_noise_sigma_(vk::param<double>(pnh_, "dataset_noise_sigma", 0.0))
  , sleep_us_(vk::param<int>(pnh_, "sleep_us", 0))
{
  // create pose tracefile
  std::string trace_est_name(svo_->options_.trace_dir + "/traj_estimate.txt");
  trace_est_pose_.open(trace_est_name.c_str());
  if(trace_est_pose_.fail())
    throw std::runtime_error("Could not create tracefile. Does folder exist?");
}

void BenchmarkNode::traceGroundtruth(const Transformation& T_w_gt, const double timestamp)
{
  const Eigen::Quaterniond& q = T_w_gt.getRotation().toImplementation();
  const Vector3d& p = T_w_gt.getPosition();
  trace_gt_pose_.precision(15);
  trace_gt_pose_.setf(std::ios::fixed, std::ios::floatfield );
  trace_gt_pose_ << timestamp << " ";
  trace_gt_pose_.precision(6);
  trace_gt_pose_ << p.x() << " " << p.y() << " " << p.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

void BenchmarkNode::tracePose(const Transformation& T_w_f, const int64_t timestamp)
{
  const Eigen::Quaterniond& q = T_w_f.getRotation().toImplementation();
  const Vector3d& p = T_w_f.getPosition();
  trace_est_pose_.precision(8);
  trace_est_pose_ << timestamp << " ";
  trace_est_pose_ << p.x() << " " << p.y() << " " << p.z() << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

void BenchmarkNode::tracePoseError(const Transformation& T_f_gt, const double timestamp)
{
  const Vector3d& et = T_f_gt.getPosition(); // translation error
  trace_trans_error_.precision(15);
  trace_trans_error_.setf(std::ios::fixed, std::ios::floatfield );
  trace_trans_error_ << timestamp << " ";
  trace_trans_error_.precision(6);
  trace_trans_error_ << et.x() << " " << et.y() << " " << et.z() << " " << std::endl;
  Vector3d er(vk::dcm2rpy(T_f_gt.getRotationMatrix())); // rotation error in roll-pitch-yaw
  trace_rot_error_.precision(15);
  trace_rot_error_.setf(std::ios::fixed, std::ios::floatfield );
  trace_rot_error_ << timestamp << " ";
  trace_rot_error_.precision(6);
  trace_rot_error_ << er.x() << " " << er.y() << " " << er.z() << " " << std::endl;
}

void BenchmarkNode::tracePoseKitti(const Transformation& T_w_f)
{
  const Eigen::Matrix4d& T = T_w_f.getTransformationMatrix();
  trace_est_pose_ << T(0,0) << " " << T(0, 1) << " " << T(0, 2) << " " << T(0, 3) << " "
                  << T(1,0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << " "
                  << T(2,0) << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 3) << " "
                  << std::endl;
}

void BenchmarkNode::traceDepthError(const FramePtr& frame, const cv::Mat& depthmap)
{
  trace_depth_error_.precision(6);
  for(size_t i = 0; i < frame->numFeatures(); ++i)
  {
    if(frame->landmark_vec_[i])
    {
      const double depth_estimated = (frame->pos() - frame->pos()).norm();
      const double depth_true = depthmap.at<float>((int) frame->px_vec_(1, i),
                                                   (int) frame->px_vec_(0, i));
      trace_depth_error_ << frame->id() << " "
                         << depth_estimated-depth_true << std::endl;
    }
  }
}

void BenchmarkNode::addNoiseToImage(cv::Mat& img, double sigma)
{
  uint8_t* p = (uint8_t*) img.data;
  uint8_t* p_end = img.ptr<uint8_t>(img.rows, img.cols);
  while(p != p_end)
  {
    int val = *p + vk::Sample::gaussian(sigma) + 0.5;
    *p = std::max(std::min(val, 255), 0);
    ++p;
  }
}

void BenchmarkNode::runBenchmark(const std::string& dataset_dir)
{
  SVO_INFO_STREAM("Run Benchmark");

  // Load imu messages.
  if(imu_handler_)
  {
    std::string imu_filename(dataset_dir + "/data/imu.txt");
    if(!imu_handler_->loadImuMeasurementsFromFile(imu_filename))
      return;
  }

  // Load images.
  std::string img_filename(dataset_dir + "/data/images.txt");
  std::ifstream img_fs(img_filename.c_str());
  if(!img_fs.is_open())
  {
    SVO_ERROR_STREAM("Could not open images file " << img_filename);
    return;
  }

  size_t first_frame_id = vk::getParam<int>("svo/dataset_first_frame", 0);
  bool trace_only_keyframes = vk::getParam<bool>("svo/trace_only_keyframes", false);
  std::map<int, int64_t, std::less<int>> frameid_timestamp_map;
  while(img_fs.good() && !img_fs.eof() && ros::ok())
  {
    if(img_fs.peek() == '#') // skip comments
      img_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // load image
    size_t img_id;
    double stamp_seconds;
    std::string img_name;
    img_fs >> img_id >> stamp_seconds >> img_name;
    img_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if(img_id < first_frame_id)
      continue;
    uint64_t stamp_nanoseconds = stamp_seconds*1e9;
    std::string img_filename(dataset_dir + "/data/" + img_name);
    cv::Mat img(cv::imread(img_filename, 0));
    if(img.empty())
    {
      SVO_ERROR_STREAM("Reading image "<<img_filename<<" failed.");
      break;
    }

    // add image to VO
    setImuPrior(stamp_nanoseconds);
    processImageBundle({img}, stamp_nanoseconds);
    publishResults({img}, ros::Time::now().toNSec());

    if(svo_->stage() == Stage::kTracking)
    {
      const FrameBundle& nframe = *svo_->getLastFrames();
      if(nframe.isKeyframe())
      {
        frameid_timestamp_map.insert(
              std::make_pair(nframe.getBundleId(), img_id));
      }
      if(!svo_->getBundleAdjuster() &&
         ((nframe.isKeyframe() && trace_only_keyframes) || !trace_only_keyframes))
        tracePose(nframe.get_T_W_B(), img_id);
    }

    // restart if it fails!
    if(svo_->stage() == Stage::kPaused)
      svo_->start();

    usleep(sleep_us_);
  }

#ifdef SVO_USE_BACKEND
  if(backend_interface_)
  {
    std::cout << "frameid_timestamp_map.size = " << frameid_timestamp_map.size() << std::endl;
    backend_visualizer_->tracePose(
          frameid_timestamp_map, backend_interface_->optimizer_->estimate_);
  }
#endif
}

void BenchmarkNode::runBlenderBenchmark(const std::string& dataset_dir, bool depthmap_has_zbuffer)
{
  SVO_INFO_STREAM("Run Blender Benchmark");

  // since initialization is done using the groundtruth for synthetic data,
  // tell the reprojector not to delete points with less than two observations
  svo_->reprojectors_.at(0)->options_.remove_unconstrained_points = false;

  // create tracefiles
  trace_gt_pose_.open(svo_->options_.trace_dir + "/groundtruth_matched.txt");
  trace_trans_error_.open(svo_->options_.trace_dir + "/translation_error.txt");
  trace_rot_error_.open(svo_->options_.trace_dir + "/orientation_error.txt");
  trace_depth_error_.open(svo_->options_.trace_dir + "/depth_error.txt");
  if(trace_trans_error_.fail() || trace_rot_error_.fail() || trace_depth_error_.fail() || trace_gt_pose_.fail())
    throw std::runtime_error("Could not create tracefile. Does folder exist?");

  // process dataset
  test_utils::SyntheticDataset dataset(dataset_dir, 0, 0);

  // set first frame
  const size_t n_pyr_levels = 5;
  FramePtr ref_frame;
  cv::Mat depthmap;
  Transformation T_w_gt;
  if(dataset.getNextFrame(n_pyr_levels, ref_frame, &depthmap))
  {
    // extract features, generate features with 3D points
    DetectorOptions fast_options;
    AbstractDetector::Ptr detector = feature_detection_utils::makeDetector(fast_options, ref_frame->cam());
    detector->detect(ref_frame);

    if(depthmap_has_zbuffer)
    {
      SVO_INFO_STREAM("Depthmap contains z-buffer values");
    }
    else
    {
      SVO_INFO_STREAM("Depthmap contains distance to camera values");
    }

    for(size_t i = 0; i < ref_frame->num_features_; ++i)
    {
      const float depth = depthmap.at<float>(ref_frame->px_vec_(1, i), ref_frame->px_vec_(0, i));
      Eigen::Vector3d landmark_ref;
      if(depthmap_has_zbuffer)
      {
        landmark_ref = ref_frame->f_vec_.col(i) / ref_frame->f_vec_(2,i) * depth;
      }
      else
      {
        SVO_DEBUG_STREAM("Depth (" << ref_frame->px_vec_(0, i) << " , " << ref_frame->px_vec_(1, i) << ") = " << depth);
        landmark_ref = depth * (ref_frame->f_vec_.col(i)).normalized();
      }
      Eigen::Vector3d landmark_world = ref_frame->T_f_w_.inverse() * landmark_ref;
      svo::PointPtr point(new svo::Point(landmark_world));
      point->addObservation(ref_frame, i);
      ref_frame->landmark_vec_[i] = point;
    }
    SVO_INFO_STREAM("Added "<<ref_frame->num_features_<<" 3d pts to the reference frame.");

    svo_->setFirstFrames({ref_frame});
    SVO_INFO_STREAM("Set reference frame.");
  }
  else
  {
    SVO_ERROR_STREAM("Could not load first frame");
    return;
  }

  // process next frames
  frame_count_ = 1;
  FramePtr cur_frame;
  while(dataset.getNextFrame(n_pyr_levels, cur_frame, nullptr) && ros::ok())
  {
    T_w_gt = cur_frame->T_f_w_.inverse();

    ++frame_count_;
    SVO_DEBUG_STREAM("Processing image " << frame_count_ << ".");

    processImageBundle({cur_frame->img()}, cur_frame->id());
    publishResults({cur_frame->img()}, ros::Time::now().toNSec());

    if(svo_->stage() != Stage::kTracking)
    {
      SVO_ERROR_STREAM("SVO failed before entire dataset could be processed.");
      break;
    }

    if(frame_count_ == 50)
      svo_->reprojectors_.at(0)->options_.remove_unconstrained_points = true;

    // Compute pose error and trace to file
    if(svo_->getLastFrames())
    {
      Transformation T_f_gt(svo_->getLastFrames()->at(0)->T_f_w_*T_w_gt);
      tracePoseError(T_f_gt, cur_frame->getTimestampSec());
      tracePose(svo_->getLastFrames()->at(0)->T_f_w_.inverse(), cur_frame->id());
      //traceDepthError(vo_->lastFrame(), depthmap);
      traceGroundtruth(T_w_gt, cur_frame->getTimestampSec());
    }

    usleep(sleep_us_);
  }
}

void BenchmarkNode::runKittiBenchmark(const std::string& dataset_dir)
{
  VLOG(1) << "Run Kitti Benchmark";
  std::string dataset(dataset_dir + "/data/images.txt");
  std::ifstream dataset_fs(dataset.c_str());
  if(!dataset_fs.is_open()) {
    std::cout << "Could not open images file: " << dataset << std::endl;
    return;
  }
  std::cout << "cam0 label = " << svo_->cams_->getCamera(0).getLabel() << std::endl;
  std::cout << "cam1 label = " << svo_->cams_->getCamera(1).getLabel() << std::endl;

  // process dataset
  while(dataset_fs.good() && !dataset_fs.eof() && ros::ok())
  {
    // skip comments
    if(dataset_fs.peek() == '#')
      dataset_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // load data
    uint64_t id;
    double stamp;
    std::string img_l_name, img_r_name;
    dataset_fs >> id >> stamp >> img_l_name >> img_r_name;
    cv::Mat img_l_8uC1(cv::imread(dataset_dir+"/data/"+img_l_name, 0));
    cv::Mat img_r_8uC1(cv::imread(dataset_dir+"/data/"+img_r_name, 0));
    if(img_l_8uC1.empty() || img_r_8uC1.empty())
    {
      SVO_ERROR_STREAM("Reading image failed: " << img_l_name);
      return;
    }
    processImageBundle({img_l_8uC1, img_r_8uC1}, id);
    publishResults({img_l_8uC1, img_r_8uC1}, ros::Time::now().toNSec());
    if(svo_->getLastFrames())
      tracePoseKitti(svo_->getLastFrames()->at(0)->T_world_cam());
  }
  VLOG(1) << "Dataset processing finished. shutting down...";
}

void BenchmarkNode::runEurocBenchmark(const std::string& dataset_dir)
{
  VLOG(1) << "Run Euroc Benchmark";
  const size_t num_cameras = ncam_->numCameras();
  std::vector<size_t> cam_indices;
  for(size_t i = 0; i < num_cameras; ++i)
    cam_indices.push_back(i);
  CsvDatasetReader dataset(dataset_dir+"/data", {0}, cam_indices, {});
  std::vector<dataset::CameraMeasurement::ConstPtr> cam_measurements(num_cameras, nullptr);
  std::map<int, int64_t, std::less<int>> frameid_timestamp_map;
  int counter = 0u;
  const int fixed_frame_rate = vk::param<int>(pnh_, "fixed_frame_rate", -1); // negative means not fixed
  const int first_frame_id = vk::param<int>(pnh_, "dataset_first_frame", 0);
  const int last_frame_id = vk::param<int>(pnh_, "dataset_last_frame", -1);
  for(const CsvDatasetReader::StampMeasurementPair& item : dataset)
  {

    const dataset::MeasurementBase::Ptr& data = item.second;
    switch (data->measurement_type)
    {
      case dataset::MeasurementType::kCamera:
      {
        dataset::CameraMeasurement::ConstPtr cam_data =
            std::dynamic_pointer_cast<const dataset::CameraMeasurement>(data);
        CHECK_LE(cam_data->camera_index, cam_measurements.size());
        cam_measurements.at(cam_data->camera_index) = cam_data;

        // We can process the measurments only if we received all camera images.
        // TODO(Cfo): check that the measurements are synchronized!
        if(std::all_of(cam_measurements.cbegin(), cam_measurements.cend(),
                       [](dataset::CameraMeasurement::ConstPtr i){ return i.get() != nullptr; }))
        {
          // Process frame-bundle.
          if(++counter >= first_frame_id)
          {
            VLOG(3) << "Euroc count: " << counter;
            vk::Timer t;

            const int64_t stamp = cam_measurements[0]->timestamp_nanoseconds;
            std::vector<cv::Mat> img_vec;
            for(const auto& m : cam_measurements)
              img_vec.push_back(m->getImage());
            setImuPrior(stamp);
            processImageBundle(img_vec, stamp);
            publishResults(img_vec, stamp);
            if(svo_->getLastFrames())
            {
              const FrameBundle& nframe = *svo_->getLastFrames();
              if(nframe.isKeyframe())
              {
                frameid_timestamp_map.insert(
                      std::make_pair(nframe.getBundleId(), nframe.getMinTimestampNanoseconds()));
                if(!svo_->getBundleAdjuster())
                  tracePose(nframe.get_T_W_B(), nframe.getMinTimestampNanoseconds());
              }
            }

            t.stop();
            if(fixed_frame_rate > 0 && svo_->stage() == Stage::kTracking)
            {
              const double time_remaining = 1.0/fixed_frame_rate - t.getTime();
              if(time_remaining > 0.0)
              {
                int sleep_ms = time_remaining*1000.0;
                VLOG(3) << "Sleep for " << sleep_ms;
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
              }
            }

            // restart if it fails!
            if(svo_->stage() == Stage::kPaused)
              svo_->start();
          }
          std::fill(cam_measurements.begin(), cam_measurements.end(), nullptr);
        }
        break;
      }

      case dataset::MeasurementType::kImu:
      {
        if(imu_handler_)
        {
          dataset::ImuMeasurement::ConstPtr imu_data =
                std::dynamic_pointer_cast<const dataset::ImuMeasurement>(data);
          const ImuMeasurement m(
                imu_data->timestamp_nanoseconds *
                common::conversions::kNanoSecondsToSeconds,
                imu_data->acc_gyro_measurement.tail<3>(),
                imu_data->acc_gyro_measurement.head<3>());

          imu_handler_->addImuMeasurement(m);
        }
        break;
      }

      default:
        LOG(FATAL) << "Unhandled message type: " << static_cast<int>(data->measurement_type);
        break;
    }

    if(last_frame_id > 0 && counter > last_frame_id)
      break;

    if(!ros::ok())
      break;
  }

#ifdef SVO_USE_BACKEND
  if(backend_interface_)
  {
    backend_visualizer_->tracePose(
          frameid_timestamp_map, backend_interface_->optimizer_->estimate_);
  }
#endif

  VLOG(1) << "Dataset processing finished. shutting down...";
}

void BenchmarkNode::runArrayBenchmark(const std::string& dataset_dir)
{
  // load images
  std::string img_filename(dataset_dir + "/data/images.txt");
  std::ifstream img_fs(img_filename.c_str());
  if(!img_fs.is_open())
  {
    SVO_ERROR_STREAM("Could not open images file " << img_filename);
    return;
  }

  size_t first_frame_id = vk::getParam<int>("svo/dataset_first_frame", 0);
  while(img_fs.good() && !img_fs.eof() && ros::ok())
  {
    if(img_fs.peek() == '#') // skip comments
      img_fs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // load image
    uint64_t stamp = 0;
    std::string imgname1, imgname2, imgname3, imgname4;
    img_fs >> stamp >> imgname1 >> imgname2 >> imgname3 >> imgname4;
    if(stamp < first_frame_id)
      continue;
    std::string();
    cv::Mat img1(cv::imread(dataset_dir + "/data/" + imgname1, 0));
    cv::Mat img2(cv::imread(dataset_dir + "/data/" + imgname2, 0));
    cv::Mat img3(cv::imread(dataset_dir + "/data/" + imgname3, 0));
    cv::Mat img4(cv::imread(dataset_dir + "/data/" + imgname4, 0));
    if(img1.empty() || img2.empty() || img3.empty() || img4.empty())
    {
      SVO_ERROR_STREAM("Reading image "<< dataset_dir << "/data/" << imgname1 <<" failed.");
      return;
    }

    // add image to VO
    processImageBundle({img1, img2, img3, img4}, stamp);
    publishResults({img1, img2, img3, img4}, stamp);

    cv::waitKey(10);
  }
}

} // namespace svo

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string benchmark_dir(vk::param<std::string>(pnh, "dataset_directory", "/tmp"));
  if(vk::param<bool>(pnh, "dataset_is_kitti", false))
  {
    svo::BenchmarkNode benchmark(svo::PipelineType::kStereo, nh, pnh);
    benchmark.runKittiBenchmark(benchmark_dir);
  }
  else if(vk::param<bool>(pnh, "dataset_is_euroc", false))
  {
    svo::PipelineType type = (vk::param<bool>(pnh, "dataset_is_stereo", true)) ?
          svo::PipelineType::kStereo : svo::PipelineType::kMono;
    svo::BenchmarkNode benchmark(type, nh, pnh);
    benchmark.runEurocBenchmark(benchmark_dir);
  }
  else if(vk::param<bool>(pnh, "dataset_is_blender", false))
  {
    // does depthmap contains z values or distance to the camera (default)
    const bool depthmap_has_zbuffer = vk::param<bool>(pnh, "depthmap_has_zbuffer", false);
    svo::BenchmarkNode benchmark(svo::PipelineType::kMono, nh, pnh);
    benchmark.runBlenderBenchmark(benchmark_dir, depthmap_has_zbuffer);
  }
  else
  {
    svo::BenchmarkNode benchmark(svo::PipelineType::kMono, nh, pnh);
    benchmark.runBenchmark(benchmark_dir);
  }

  SVO_INFO_STREAM("BenchmarkNode finished.");
  return 0;
}
