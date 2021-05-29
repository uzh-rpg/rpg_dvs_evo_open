// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#include <algorithm>
#include <glog/logging.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/patch_normal.h>
#include <opencv2/highgui/highgui.hpp>

namespace svo {

constexpr int PatchNormal::patch_halfsize_;
constexpr int PatchNormal::patch_size_;
constexpr int PatchNormal::patch_area_;

PatchNormal::PatchNormal(const SolverOptions& solver_options)
  : vk::solver::MiniLeastSquaresSolver<2, Vector2d, PatchNormal>(solver_options)
  , display_(true)
{
  img_residuals_ = cv::Mat(patch_size_, patch_size_, CV_32F);
}

PatchNormal::SolverOptions PatchNormal::getDefaultSolverOptions()
{
  SolverOptions options;
  options.strategy = vk::solver::Strategy::GaussNewton;
  options.max_iter = 10;
  options.eps = 0.000001;
  return options;
}

void PatchNormal::run(
    FeaturePtr ftr_ref,
    double depth,
    const FramePtr& frame_cur,
    Vector3d& normal_ref,
    Matrix2d& information)
{
  CHECK(ftr_ref);
  CHECK(!ftr_ref->frame.expired());
  CHECK(frame_cur);

  reset();

  have_ref_patch_cache_ = false;
  ftr_ref_ = ftr_ref;
  frame_cur_ = frame_cur;
  depth_ = depth;
  frame_ref_ = ftr_ref_->frame.lock();
  CHECK(frame_ref_) << "could not acquire weak_ptr";
  normal_ = normal_ref;
  T_c_r_ = frame_cur_->T_f_w_*frame_ref_->T_f_w_.inverse();
  n_meas_ = 0;

  //setPrior(Vector2d::Zero(), information*0.25); // prior is zero because no update

  // init orthogonal update vector
  Vector3d tmp = Vector3d::UnitX();
  if(normal_.y() < normal_.x())
    tmp = Vector3d::UnitY();
  update_x_vec_ = normal_.cross(tmp);
  update_y_vec_ = normal_.cross(update_x_vec_);
  update_x_vec_.normalize();
  update_y_vec_.normalize();

  std::cout << "optimize" << std::endl;
  Vector2d params = Vector2d::Zero();
  optimize(params);

  // write output parameters
  normal_ref = normal_+update_x_vec_*params[0]+update_y_vec_*params[1];
  normal_ref.normalize();
  information = H_;
}

void PatchNormal::precomputeJacobian()
{
  const cv::Mat& img_ref = frame_ref_->img();
  const double focal_length = frame_ref_->getErrorMultiplier();

  // create homography matrix
  const Vector3d p = ftr_ref_->f*depth_;
  //const Vector3d p_dir = p.normalized();
  const Vector3d t = (frame_ref_->T_f_w_*frame_cur_->pos());
  const Matrix3d A = (normal_.dot(p)*Matrix3d::Identity() - t*normal_.transpose());
  const Matrix3d A_inv = A.inverse();

  // projection derivative
  Matrix<double,2,3> J_proj;
  J_proj << 1, 0, -p[0]/p[2],
            0, 1, -p[1]/p[2];

  // derivative of A w.r.t, z
  //const Matrix3d A_deriv_z = Matrix3d::Identity() * p_dir.dot(normal_);

  // derivative of A w.r.t. orienatation change around x, y
  const Matrix3d A_deriv_x = Matrix3d::Identity() * p.dot(update_x_vec_) - t*update_x_vec_.transpose();
  const Matrix3d A_deriv_y = Matrix3d::Identity() * p.dot(update_y_vec_) - t*update_y_vec_.transpose();

  Vector2d px_ref_origin(ftr_ref_->px-Vector2d(patch_halfsize_, patch_halfsize_));
  for(int y=0; y<patch_size_; ++y)
  {
    for(int x=0; x<patch_size_; ++x)
    {
      const Vector2d px_ref = px_ref_origin+Vector2d(x,y);
      Vector3d f_ref;
      frame_ref_->cam()->backProject3(px_ref, &f_ref);
      f_ref.normalize();

      Matrix<double,3,2> A_deriv;
      //Matrix3d A_deriv;
      //A_deriv.col(0) = A_deriv_z*f_ref;
      A_deriv.col(0) = A_deriv_x*f_ref;
      A_deriv.col(1) = A_deriv_y*f_ref;

      //const Matrix<double,2,3> J = focal_length*J_proj*A_inv*A_deriv;
      const Matrix2d J = focal_length*J_proj*(-A_inv)*A_deriv;

      float dx = 0.5*(vk::interpolateMat_8u(img_ref, px_ref[0]+1, px_ref[1])
                     -vk::interpolateMat_8u(img_ref, px_ref[0]-1, px_ref[1]));
      float dy = 0.5*(vk::interpolateMat_8u(img_ref, px_ref[0], px_ref[1]+1)
                     -vk::interpolateMat_8u(img_ref, px_ref[0], px_ref[1]-1));

      jacobian_cache_.col(y*patch_size_+x) = -(J.row(0)*dx + J.row(1)*dy);
    }
  }
  have_ref_patch_cache_ = true;
}

double PatchNormal::evaluateError(
    const State& params,
    HessianMatrix* H,
    GradientVector* g)
{
  if(!have_ref_patch_cache_)
    precomputeJacobian();

  // compute the weights on the first iteration
  /*
  std::vector<float> errors;
  if(compute_weight_scale)
    errors.reserve(patch_area_);
  */

  double depth = depth_; //+params[0]; //*
  //Vector3d normal = normal_+update_x_vec_*params[1]+update_y_vec_*params[2];
  Vector3d normal = normal_+update_x_vec_*params[0]+update_y_vec_*params[1];
  //normal.normalize();

  float chi2 = 0.0;
  const CameraPtr cam_ref = frame_ref_->cam_;

  const double I = 0.01; //DiagonalMatrix<double,3,3> I(0.01, 0.01, 0.01); // 1/(pixel noise)^2

  // compute current homography
  Vector3d t = (frame_ref_->T_f_w_*frame_cur_->pos());
  const Matrix3d H_c_r = T_c_r_.getRotationMatrix()*
      (normal.dot(ftr_ref_->f*depth)*Matrix3d::Identity()
       - t*normal.transpose());

  const Vector2d px_origin(ftr_ref_->px-Vector2d(patch_halfsize_, patch_halfsize_));
  const cv::Mat& img_ref = frame_ref_->img_pyr_[0];
  const cv::Mat& img_cur = frame_cur_->img_pyr_[0];
  const CameraPtr cam_cur = frame_cur_->cam_;
  for(int y=0; y<patch_size_; ++y)
  {
    for(int x=0; x<patch_size_; ++x)
    {
      // apply homography to find corresponding point
      const Vector2d px_ref(px_origin+Vector2d(x,y));
      Vector3d f_ref;
      cam_ref->backProject3(px_ref, &f_ref);
      Vector3d f_cur(H_c_r*f_ref);
      Vector2d px_cur;
      cam_cur->project3(f_cur, &px_cur);

      // check if the projected point lies within the image boundaries
      if(px_cur[0]<1 || px_cur[1]<1 || px_cur[0]+2>=img_cur.cols || px_cur[1]+2>=img_cur.rows)
        continue;

      // compute residual
      const float intensity_ref = img_ref.at<uint8_t>((int) px_ref[1], (int) px_ref[0]);
      const float intensity_cur = vk::interpolateMat_8u(img_cur, px_cur[0], px_cur[1]);
      const float e = intensity_cur - intensity_ref;

      // used to compute scale for robust cost
      /*
      if(compute_weight_scale)
         errors.push_back(fabsf(e));
      */

      // robustification
      float weight = 1.0;
      /*
      if(use_weights_) {
        weight = weight_function_->weight(e/scale_);
      }
      */

      chi2 += e*e*weight;
      n_meas_++;

      if(display_)
        img_residuals_.at<float>(y,x) = e;

      if(H && g)
      {
        Vector2d J = jacobian_cache_.col(y*patch_size_+x);
        *H += J*I*J.transpose()*weight;
        *g -= I*J*e*weight;
      }
    }
  }

  // compute the weights on the first iteration
  /*
  if(compute_weight_scale && iter_ == 0)
    scale_ = scale_estimator_->compute(errors);
  */

  return chi2/(n_meas_*n_meas_);
}

void PatchNormal::update(
    const State& param_old,
    const UpdateVector& dx,
    State& param_new)
{
  param_new = param_old+dx;
}

void PatchNormal::applyPrior(const State& current_model)
{
  H_.noalias() += I_prior_;
  g_.noalias() += I_prior_*(current_model-prior_);
}

void PatchNormal::startIteration()
{}

void PatchNormal::finishIteration()
{
  if(display_)
  {
    cv::imshow("residuals", img_residuals_/255*10);
    cv::waitKey(0);
  }
}

} // namespace svo

