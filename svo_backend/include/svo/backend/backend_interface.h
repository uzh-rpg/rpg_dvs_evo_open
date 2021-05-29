#pragma once

#include <svo/abstract_bundle_adjustment.h>
#include <svo/backend/backend_types.h>
#include <vikit/performance_monitor.h>

namespace svo {

// fwd
class BackendOptimizer;
class GraphManager;
class ImuHandler;
struct GraphManagerOptions;
struct OptimizerBackendOptions;

struct BackendInterfaceOptions
{
  std::string trace_dir = "/tmp";
  int isam_wait_time_ms = 2;            ///< Time we wait for iSAM to release lock.
  bool add_imu_factors = false;
  size_t n_frames_in_init_ba = 8u;
  bool use_smart_factors = false;
  size_t min_num_obs = 2u;
};

enum class EstimatorState {
  kUninitialized,
  kInitialBundleAdjustment,
  kRunning
};

class BackendInterface : public AbstractBundleAdjustment
{
public:

  typedef std::shared_ptr<BackendInterface> Ptr;

  BackendInterfaceOptions options_;

  // modules
  std::shared_ptr<BackendOptimizer> optimizer_;
  std::shared_ptr<GraphManager> graph_;
  std::shared_ptr<ImuHandler> imu_handler_;

  // state
  EstimatorState estimator_state_ = EstimatorState::kUninitialized;
  ViNodeState last_state_;
  BundleId last_added_index_;
  int64_t last_added_frame_stamp_ns_;
  BundleId last_loaded_estimate_index_;
  size_t num_frames_in_backend_ = 0u;

  BackendInterface(
      const BackendInterfaceOptions& options,
      const GraphManagerOptions& graph_manager_options,
      const OptimizerBackendOptions& optimizer_options);

  ~BackendInterface() = default;

  /// Invoke bundle adjustment.
  virtual void bundleAdjustment(
      const FrameBundlePtr& frame_bundle) override;

  /// Update map with results from bundle adjustment.
  virtual void loadMapFromBundleAdjustment(
      const FrameBundlePtr& new_frames,
      const FrameBundlePtr& last_frames,
      const MapPtr& map) override;

  /// Reset bundle adjustment
  virtual void reset() override;

  /// Bundle adjustment can run completely in parallel. Start the thread to do so.
  virtual void startThread() override;

  /// Stop and join the bundle adjustment thread
  virtual void quitThread() override;


protected:

  void initializeBackend();

  void addVisualMeasurementsToGraph(
      const FrameBundlePtr& frame_bundle);

  void addInertialMeasurementsToGraph(
      const FrameBundlePtr& frame_bundle);

  void addInitialStateAndPriorsToGraph(
      const BundleId& state_index,
      const ViNodeState& state);
};

} // namespace svo
