// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#pragma once

// svo
#include <svo/global.h>

namespace svo
{

/// EXPERIMENTAL Defines interface for various bundle adjustment methods
class AbstractBundleAdjustment
{
public:

  typedef std::shared_ptr<AbstractBundleAdjustment> Ptr;

  /// Default constructor.
  AbstractBundleAdjustment() {}

  virtual ~AbstractBundleAdjustment() {}

  // no copy
  AbstractBundleAdjustment(const AbstractBundleAdjustment&) = delete;
  AbstractBundleAdjustment& operator=(const AbstractBundleAdjustment&) = delete;

  /// Invoke bundle adjustment.
  virtual void bundleAdjustment(
      const FrameBundlePtr& frame_bundle) = 0;

  /// Update map with results from bundle adjustment.
  virtual void loadMapFromBundleAdjustment(
      const FrameBundlePtr& new_frames,
      const FrameBundlePtr& last_frames,
      const MapPtr& map) = 0;

  /// Reset bundle adjustment
  virtual void reset() = 0;

  /// Bundle adjustment can run completely in parallel. Start the thread to do so.
  virtual void startThread() = 0;

  /// Stop and join the bundle adjustment thread
  virtual void quitThread() = 0;
};

} // namespace svo
