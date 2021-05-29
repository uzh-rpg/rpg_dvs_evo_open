// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).

#ifdef SVO_USE_ROS
# include <vikit/params_helper.h>
# include <ros/package.h>
#endif
#include <svo/config.h>
#include <vikit/timer.h>

namespace svo {

Config::Config() :
#ifdef SVO_USE_ROS
  trace_name(vk::getParam<string>("svo/trace_name", "svo_"+vk::Timer::getCurrentTimeStr())),
  trace_dir(vk::getParam<string>("svo/trace_dir", ros::package::getPath("svo")+"/trace")),
  n_pyr_levels(vk::getParam<int>("svo/n_pyr_levels", 3)),
  use_imu(vk::getParam<bool>("svo/use_imu", false)),
  core_n_kfs(vk::getParam<int>("svo/core_n_kfs", 3)),
  map_scale(vk::getParam<double>("svo/map_scale", 1.0)),
  grid_size(vk::getParam<int>("svo/grid_size", 30)),
  init_min_disparity(vk::getParam<double>("svo/init_min_disparity", 50.0)),
  init_min_tracked(vk::getParam<int>("svo/init_min_tracked", 50)),
  init_min_inliers(vk::getParam<int>("svo/init_min_inliers", 40)),
  init_min_features(vk::getParam<int>("svo/init_min_features", 100)),
  init_min_depth_error(vk::getParam<double>("svo/init_min_depth_error", 1.)),
  init_use_att_and_depth(vk::getParam<bool>("svo/init_use_att_and_depth", false)),
  klt_max_level(vk::getParam<int>("svo/klt_max_level", 4)),
  klt_min_level(vk::getParam<int>("svo/klt_min_level", 2)),
  klt_max_iter(vk::getParam<int>("svo/klt_max_iter", 30)),
  reproj_thresh(vk::getParam<double>("svo/reproj_thresh", 2.0)),
  poseoptim_thresh(vk::getParam<double>("svo/poseoptim_thresh", 2.0)),
  poseoptim_num_iter(vk::getParam<int>("svo/poseoptim_num_iter", 10)),
  structureoptim_max_pts(vk::getParam<int>("svo/structureoptim_max_pts", 20)),
  structureoptim_num_iter(vk::getParam<int>("svo/structureoptim_num_iter", 5)),
  loba_thresh(vk::getParam<double>("svo/loba_thresh", 2.0)),
  loba_robust_huber_width(vk::getParam<double>("svo/loba_robust_huber_width", 1.0)),
  loba_num_iter(vk::getParam<int>("svo/loba_num_iter", 0)),
  kfselect_min_dist(vk::getParam<double>("svo/kfselect_min_dist", 0.12)),
  kfselect_min_fts(vk::getParam<int>("svo/kfselect_min_fts", 80)),
  kfselect_min_frames(vk::getParam<int>("svo/kfselect_min_frames", 10)),
  triang_min_corner_score(vk::getParam<double>("svo/triang_min_corner_score", 20.0)),
  triang_half_patch_size(vk::getParam<int>("svo/triang_half_patch_size", 4)),
  subpix_n_iter(vk::getParam<int>("svo/subpix_n_iter", 10)),
  max_n_kfs(vk::getParam<int>("svo/max_n_kfs", 10)),
  img_imu_delay(vk::getParam<double>("svo/img_imu_delay", 0.0)),
  limit_n_fts(vk::getParam<bool>("svo/limit_n_fts", true)),
  max_fts(vk::getParam<int>("svo/max_fts", 120)),
  quality_min_fts(vk::getParam<int>("svo/quality_min_fts", 50)),
  quality_max_drop_fts(vk::getParam<int>("svo/quality_max_drop_fts", 40)),
  use_edgelets(vk::getParam<bool>("svo/use_edgelets", false)),
  relocalization_max_trials(vk::getParam<int>("svo/relocalization_max_trials", 100)),
  img_align_halfpatch_size(vk::getParam<int>("svo/img_align_halfpatch_size", 2)),
  img_align_prior_lambda(vk::getParam<double>("svo/img_align_prior_lambda", 2.0)),
  poseoptim_prior_lambda(vk::getParam<double>("svo/poseoptim_prior_lambda", 2.0)),
  features_min_level(vk::getParam<int>("svo/features_min_level", 0))
#else
  trace_name("svo_"+vk::Timer::getCurrentTimeStr()),
  trace_dir("/tmp"),
  n_pyr_levels(4),
  use_imu(false),
  core_n_kfs(3),
  map_scale(1.0),
  grid_size(40),
  init_min_disparity(50.0),
  init_min_tracked(50),
  init_min_inliers(40),
  init_min_features(100),

  init_min_depth_error(0.02), // in [m] 2cm +-
  init_baseline_thresholding(false),
  init_expected_depth(1.0), // in [m]

  init_use_att_and_depth(false),
  klt_max_level(5),
  klt_min_level(3),
  klt_max_iter(30),
  reproj_thresh(2.0),
  poseoptim_thresh(2.0),
  poseoptim_num_iter(10),
  structureoptim_max_pts(20),
  structureoptim_num_iter(5),
  loba_thresh(2.0),
  loba_robust_huber_width(1.0),
  loba_num_iter(20),
  kfselect_min_dist(0.3/*12*/),
  kfselect_min_fts(80),
  kfselect_min_frames(10),
  triang_min_corner_score(20.0),
  triang_half_patch_size(4),
  subpix_n_iter(10),
  max_n_kfs(0),
  img_imu_delay(0.0),
  limit_n_fts(true),
  max_fts(300),
  quality_min_fts(70),
  quality_max_drop_fts(100),
  use_edgelets(true),
  relocalization_max_trials(100),
  img_align_halfpatch_size(4), // patch size for sparse image align
  img_align_prior_lambda(2.0),
  poseoptim_prior_lambda(2.0),
  features_min_level(1),
  max_fts_in_img_align_(0)
#endif
{}

Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

} // namespace svo

