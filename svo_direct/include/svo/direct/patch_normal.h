// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#pragma once

#include <vikit/solver/mini_least_squares_solver.h>

namespace vk {
class AbstractCamera;
}

namespace svo {

class Frame;
class Point;
struct Feature;

/// Estimate the patch normal from multiple observations of the corresponding
/// 3D point.
class PatchNormal : public vk::solver::MiniLeastSquaresSolver<2, Eigen::Vector2d, PatchNormal>
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using SolverOptions = vk::solver::MiniLeastSquaresSolverOptions;

  static constexpr int patch_halfsize_ = 20;
  static constexpr int patch_size_ = 2*patch_halfsize_;
  static constexpr int patch_area_ = patch_size_*patch_size_;

  PatchNormal(const SolverOptions& solver_options);
  ~PatchNormal() = default;

  static SolverOptions getDefaultSolverOptions();

  void run(
      std::shared_ptr<Feature> ftr_ref,
      double depth,
      const FramePtr& frame_cur,
      Eigen::Vector3d& normal_ref,
      Eigen::Matrix2d& information);

  void precomputeJacobian();

//protected:
  bool display_;
  cv::Mat img_residuals_;
  FeaturePtr ftr_ref_;
  double depth_;                  //!< depth of the point in the reference frame
  FramePtr frame_ref_;              //!< reference frame
  FramePtr frame_cur_;

  Transformation T_c_r_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d update_x_vec_;
  Eigen::Vector3d update_y_vec_;

  // cache:
  Eigen::Matrix<double, 2, patch_area_> jacobian_cache_;
  bool have_ref_patch_cache_;

  double evaluateError(
      const State& params,
      HessianMatrix* H,
      GradientVector* g);

  void update(
      const State& param_old,
      const UpdateVector& dx,
      State& param_new);

  void applyPrior(const State& current_model);

  void startIteration();

  void finishIteration();
};

} // namespace svo
