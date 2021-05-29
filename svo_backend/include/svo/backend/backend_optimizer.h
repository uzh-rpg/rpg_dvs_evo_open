#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <unordered_map>

#include <svo/common/types.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

namespace svo {

// fwd
class GraphManager;

struct OptimizerBackendOptions
{
  size_t max_iterations_per_update = 10;
  bool verbose = false;
  bool output_errors = false;
  std::string trace_dir = "/tmp";
  double isam_relinearize_thresh = 0.1;
  double isam_relinearize_skip = 10.0;
  double isam_wildfire_thresh = 0.001;
  bool isam_detailed_results = false;
};

class BackendOptimizer
{
public:
  typedef std::shared_ptr<BackendOptimizer> Ptr;

  BackendOptimizer(
      const OptimizerBackendOptions& options,
      std::shared_ptr<GraphManager>& graph);

  virtual ~BackendOptimizer();

  void reset();

  void initialize();

  void optimize();

  OptimizerBackendOptions options_;
  std::shared_ptr<gtsam::ISAM2> isam_;
  std::shared_ptr<GraphManager> graph_;

  // last computed estimate
  std::mutex estimate_mut_;
  gtsam::Values estimate_;
  BundleId estimate_state_index_ = -1; // Id of last state in estimate.

  // thread
  std::mutex optimizer_cond_var_mut_;
  std::condition_variable optimizer_condition_var_;
  std::shared_ptr<std::thread> thread_;
  bool quit_thread_ = false;
  void startThread();
  void quitThread();

// protected -------------------------------------------------------------------

  void threadLoop();
  bool optimizeImpl();
};

} // namespace svo
