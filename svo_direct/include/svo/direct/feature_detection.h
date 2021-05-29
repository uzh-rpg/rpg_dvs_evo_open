// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#pragma once

#include <svo/common/types.h>
#include <svo/common/camera_fwd.h>
#include <svo/common/occupancy_grid_2d.h>
#include <svo/direct/feature_detection_types.h>

namespace svo {

//------------------------------------------------------------------------------
/// All detectors should derive from this abstract class.
class AbstractDetector
{
public:
  typedef std::shared_ptr<AbstractDetector> Ptr;

  DetectorOptions options_;

  /// Default constructor.
  AbstractDetector(
      const DetectorOptions& options,
      const CameraPtr& cam);

  /// Default destructor.
  virtual ~AbstractDetector() = default;

  // no copy
  AbstractDetector& operator=(const AbstractDetector&) = delete;
  AbstractDetector(const AbstractDetector&) = delete;

  void detect(const FramePtr &frame);

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) = 0;

  inline void resetGrid()
  {
    grid_.reset();
  }

  OccupandyGrid2D grid_;
};

//------------------------------------------------------------------------------
/// FAST detector by Edward Rosten.
class FastDetector : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~FastDetector() = default;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};

//------------------------------------------------------------------------------
/// Detect pixels that have a high gradient magnitude over multiple pyramid levels.
/// These gradient pixels are good for camera tracking.
class GradientDetector : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~GradientDetector() = default;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};


//------------------------------------------------------------------------------
/// Detect pixels that have a high gradient magnitude over multiple pyramid levels.
/// These gradient pixels are good for camera tracking.
class GradientDetectorGrid : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~GradientDetectorGrid() = default;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};

//------------------------------------------------------------------------------
/// @todo
class FastGradDetector : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~FastGradDetector() = default;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};

//------------------------------------------------------------------------------
/// Dummy detector that selects all pixels
class AllPixelsDetector : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~AllPixelsDetector() = default;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};

//------------------------------------------------------------------------------
/// Detect pixels that have strong gradients according to the paper
/// Huang, J. and Mumford, D. (1999). Statistics of natural images and models. (CVPR)
class GradientHuangMumfordDetector : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~GradientHuangMumfordDetector() = default;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};

class CannyDetector : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~CannyDetector() = default;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};

class SobelDetector : public AbstractDetector
{
public:
  using AbstractDetector::AbstractDetector; // default constructor
  virtual ~SobelDetector() = default;

  virtual void detect(
      const ImgPyr& img_pyr,
      const cv::Mat& mask,
      const size_t max_n_features,
      Keypoints& px_vec,
      Scores& score_vec,
      Levels& level_vec,
      Gradients& grad_vec,
      FeatureTypes& types_vec) override;
};

} // namespace svo
