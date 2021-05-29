// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

// svo
#include <svo/backend/backend_optimizer.h>
#include <svo/backend/graph_manager.h>
#include <svo/common/frame.h>

// boost
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// gtsam
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/linearExceptions.h>

namespace svo
{

BackendOptimizer::BackendOptimizer(
    const OptimizerBackendOptions& options,
    std::shared_ptr<GraphManager>& graph)
  : options_(options)
  , graph_(graph)
{}

BackendOptimizer::~BackendOptimizer()
{
  quitThread();
}

void BackendOptimizer::reset()
{
  VLOG(3) << "Backend: Optimizer reset";
  estimate_state_index_ = -1;
  estimate_.clear();
  isam_.reset();
}

void BackendOptimizer::initialize()
{
  VLOG(3) << "Backend: Optimizer init";
  gtsam::ISAM2GaussNewtonParams gauss_newton_params;
  gauss_newton_params.setWildfireThreshold(options_.isam_wildfire_thresh);

  // Initialize iSAM2
  gtsam::ISAM2Params isam_param;
  isam_param.optimizationParams = gauss_newton_params;
  isam_param.relinearizeThreshold = options_.isam_relinearize_thresh;
  isam_param.relinearizeSkip = options_.isam_relinearize_skip;
  isam_param.enableDetailedResults = options_.isam_detailed_results; // TODO
  isam_param.factorization = gtsam::ISAM2Params::QR;
  //isam_param.evaluateNonlinearError = true; // TODO only for debug
  isam_ = std::make_shared<gtsam::ISAM2>(isam_param);
}

void BackendOptimizer::optimize()
{
  // optimize
  if(thread_ == nullptr)
  {
    optimizeImpl();
  }
  else
  {
    optimizer_condition_var_.notify_one(); // notify the optimizer thread
  }
}

bool BackendOptimizer::optimizeImpl()
{
  CHECK_NOTNULL(isam_.get());

  // ---------------------------------------------------------------------------
  // get copy of graph and value updates so the other thread is not
  // blocked during the isam update.
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_states;
  std::vector<size_t> delete_slots;
  std::vector<int> smart_factor_point_ids;
  BundleId last_added_state_index = -1;
  {
    std::lock_guard<std::mutex> lock(graph_->mut_);
    graph_->getUpdatesCopy(
          new_factors, new_states, delete_slots, smart_factor_point_ids);
    last_added_state_index = graph_->last_added_state_index_;
  }
  if(new_factors.empty() && new_states.empty() && delete_slots.empty())
    return true;

  // ---------------------------------------------------------------------------
  // compute update
  gtsam::ISAM2Result result;
  size_t n_iter = 0;
  VLOG(4) << "iSAM2 update with " << new_factors.size() << " graph updates" <<
              " and " << new_states.size() << " new values " <<
              " and " << delete_slots.size() << " delete indices";
  try {
    result = isam_->update(new_factors, new_states, delete_slots);
  } catch(const gtsam::IndeterminantLinearSystemException& e)
  {
    std::cerr << e.what() << std::endl;
    gtsam::Key var = e.nearbyVariable();
    gtsam::Symbol symb(var);
    std::cout << "Variable has type '" << symb.chr() << "' "
              << "and index " << symb.index() << std::endl;
    throw;
  }
  ++n_iter;
  {
    std::lock_guard<std::mutex> lock(graph_->mut_);
    graph_->updateFactorSlots(result.newFactorsIndices, smart_factor_point_ids);
  }

  for(; n_iter < options_.max_iterations_per_update; ++n_iter)
  {
    try {
      result = isam_->update();
    } catch(const gtsam::IndeterminantLinearSystemException& e)
    {
      std::cerr << e.what() << std::endl;
      gtsam::Key var = e.nearbyVariable();
      gtsam::Symbol symb(var);
      std::cout << "Variable has type '" << symb.chr() << "' "
                << "and index " << symb.index() << std::endl;
      throw;
    }
  }

  // ---------------------------------------------------------------------------
  // copy estimate of the system
  {
    std::lock_guard<std::mutex> lock(estimate_mut_);
    estimate_ = isam_->calculateEstimate();
    estimate_state_index_ = last_added_state_index;
    if(options_.output_errors)
      VLOG(40) << "Final Error = " << isam_->getFactorsUnsafe().error(estimate_);
  }
  return true;
}

void BackendOptimizer::startThread()
{
  VLOG(3) << "BA: Started bundle adjustment thread";
  thread_ = std::make_shared<std::thread>(&BackendOptimizer::threadLoop, this);
}

void BackendOptimizer::quitThread()
{
  if(thread_)
  {
    quit_thread_ = true;
    optimizer_condition_var_.notify_one();
    thread_->join();
  }
}

void BackendOptimizer::threadLoop()
{
  while(!quit_thread_)
  {
    // optimize returns true when we have finished optimizing
    // it returns false, when we have to run another iteration
    if(optimizeImpl())
    {
      VLOG(30) << "Optimizer thread waiting ...";
      std::unique_lock<std::mutex> lock(optimizer_cond_var_mut_);
      optimizer_condition_var_.wait(lock);
      VLOG(30) << "Optimizer thread finished waiting.";
    }
  }
}

} // namespace svo
