#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <boost/multi_array.hpp>
#include <dvs_depth_from_defocus/depth_vector.hpp>

#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"
#include "evo_utils/camera.hpp"
#include "evo_utils/geometry.hpp"

// #define MAPPING_PERF  ///< to easily measure code for time of execution

namespace depth_from_defocus {

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef float Vote;

/**
 * State of the mapping algorithm, which define the rensponse to remote_key
 * commands
 *
 * IDLE:
 * "reset", "enable_map_expansion" or "bootstrap" -> state transition to MAPPING
 * Other commands are ignored
 *
 * MAPPING: listening to incoming events and mapping them
 * "update" -> updates and publishes the map, and clear event queue
 * @see resetVoxelGrid, processEventQueue, synthesizeAndPublishDepth,
 * publishGlobalMap and clearEventQueue
 * "reset" -> cleans map and reset the mapping thread
 * @see resetMapper
 * "disable_map_expansion" -> state transition to IDLE (no more map updates) and
 * resets map
 * @see resetMapper
 *
 * This is implemented through the remote key callback @see remoteKeyCallback
 */
enum MapperState { IDLE, MAPPING };

/**
 * Mapping node
 */
class DepthFromDefocusNode {
   public:
    /**
     * Constructor of the DepthFromDefocusNode
     *
     * Load parameters and setup the voxel grid, precomputing useful values
     */
    DepthFromDefocusNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,
                         const image_geometry::PinholeCameraModel &cam);

    /**
     * Publishes the final voxel grid, the depth map and the global map
     */
    ~DepthFromDefocusNode();

    /**
     * Processes incoming events
     *
     * Store events in the event queue and set first pose to origin, timestamped
     * as the first event arrived
     *
     * @param event_array Events array to process and store
     */
    void processEventArray(const dvs_msgs::EventArray::ConstPtr &event_array);

    /**
     * For each transform published, mark the last event for which the pose
     * is known (newest_tracked_event_)
     *
     *     ROS params:
     *         dvs_frame_id) frame id to consider after the bootstrapping
     *         dvs_bootstrap_frame_id) frame id to consider during bootstrapping
     *         world_frame_id) world frame id to consider
     *         auto_trigger) whether to automatically trigger the first map
     *             update, when enough events to create a keyframe are collected
     *
     * @param tf_msg Transform message
     */
    void tfCallback(const tf::tfMessage::ConstPtr &tf_msg);

    /**
     * Callback to remote_key commands
     *
     * Implements the state machine described in @see MapperState
     *
     * @param msg String message representing the command received
     */
    void remoteKeyCallback(const std_msgs::String::ConstPtr &msg);

    /**
     * Update camera info on callback
     *
     *
     * topic: "camera_info"
     *
     * @param msg Camera info msg
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    /**
     * Switch from bootstrap_frame_id_ to frame_id_
     *
     *     ROS params:
     *          dvs_frame_id) frame id to consider after the bootstrapping
     *          dvs_bootstrap_frame_id) frame id to consider during
     *              bootstrapping
     *          world_frame_id) world frame id to consider
     *
     * @param msg Message telling whether to switch
     */
    void copilotCallback(const std_msgs::Bool &msg);

   private:
    /**
     * Clear events, transformations, point clouds and voxel grid
     */
    void resetMapper();

    /**
     * Backproject events from the frame centers and vote the intersecting voxel
     * grids
     *
     * The events are evenly divided between the frame centers, and
     * backprojected to intersect the voxel grid The voxel are then voted
     * accordingly to @see voteForCellBilinear.
     *
     * @param events collected events
     * @param centers frame centers from which the events have been collected
     */
    void projectEventsToVoxelGrid(const std::vector<Eigen::Vector4f> &events,
                                  const std::vector<Eigen::Vector3f> &centers);

    /**
     * Clear voxel grid and set current_event_ to mark from which event in the
     * history of events the algorithm has to start to build a new map voxel
     * grid
     */
    void resetVoxelGrid();

    /**
     * Process event queue and update voxel grid
     *
     * When there is a minimum number of events to process (min_batch_size_),
     * process events in event queue and update the voxel grid. Events are
     * eventually skipped if reset is true (skip_batches_reset_,
     * skip_batches_normal_). If not, the number of batches to skip is
     * also adapted so that the events rate is smaller than max_event_rate_.
     *
     * The events are grouped in batches, and the position
     * of the camera for that batch is assumed to be the position at the middle
     * of the timespan of the batch. These poses are the centers of the
     * projectEventsToVoxelGrid.
     *
     * For each of these batches, the events are projected (homography) to the
     * reference frame and the voxel grid is updated accordingly.
     *
     *     ROS params:
     *         skip_batches) amount of batches of events to skip during
     *              optimization
     *         skip_batches_for_reset) same as skip_batches when reset = true
     *         max_event_rate) max # of events, eventually drops events to
     *              ensure this
     *         frame_size) number of events to aggregate when computing new map
     *         min_batch_size) minimum number of new events required for a map
     *              update
     *
     * @param reset Whether the "reset" command has been sent
     *
     * @see projectEventsToVoxelGrid
     */
    void processEventQueue(bool reset = false);

    /**
     * Retrieve the pose of the camera frame with respect to the world frame at
     * time t
     *
     * @param t time at which the pose has to be retrieved
     * @param T [output] pose at time t
     *
     * @return whether it is possible to convert from world frame to camera
     * frame
     */
    bool getPoseAt(const ros::Time &t, evo_utils::geometry::Transformation &T);

    /**
     * Load params related to the voxel grid and allocate memory for it
     *
     *     ROS params:
     *         min_depth) voxel grid minimum depth
     *         max_depth) voxel grid maximum depth
     *         num_depth_cells) number of depth cells
     *         adaptive_threshold_kernel_size) size of the adaptive threshold
     *             filter
     *         size adaptive_threshold_c) constant offset of the adaptive
     *             thresholding
     */
    void setupVoxelGrid();

    /**
     * Increase the vote counter of a specific cell in the vote grid
     *
     * The coordinates x_f and y_f are converted to the nearest discrete cell.
     * If the cell exists (not out of bound), the vote counter at that cell is
     * increased. If the cell does not exist, nothing happens.
     *
     * @param x_f location of the vote in the continuous horizontal axis
     * @param y_f location of the vote in the continuour vertical axis
     * @param grid the grid in which to increase the vote
     */
    void voteForCell(const float x_f, const float y_f, Vote *grid) {
        const int x = (int)(x_f + 0.5), y = (int)(y_f + 0.5);

        if (x >= 0 && x < virtual_width_ && y >= 0 && y < virtual_height_) {
            grid[x + y * virtual_width_] += 1.f;
        }
    }

    /**
     * Increase the vote counter in the vote grid acoordingly to a bilinear
     * interpolation of the continuous coordinates
     *
     * The coordinates are used to increase proportionally all the four
     * discrete locations near to the continuous space location, dividing the
     * single vote based on their distance from it.
     * If at least one of these four cells do not exist, the vote is not
     * incremented.
     *
     * @param x_f location of the vote in the continuous horizontal axis
     * @param y_f location of the vote in the continuour vertical axis
     * @param grid the grid in which to increase the votes
     */
    void voteForCellBilinear(const float x_f, const float y_f, Vote *grid) {
        if (x_f >= 0.f && y_f >= 0.f) {
            const int x = x_f, y = y_f;
            if (x + 1 < virtual_width_ && y + 1 < virtual_height_) {
                Vote *g = grid + x + y * virtual_width_;
                const float fx = x_f - x, fy = y_f - y, fx1 = 1.f - fx,
                            fy1 = 1.f - fy;

                g[0] += fx1 * fy1;
                g[1] += fx * fy1;
                g[virtual_width_] += fx1 * fy;
                g[virtual_width_ + 1] += fx * fy;
            }
        }
    }

    /**
     * Computes and stores the rectification of each pixels of the cam_, to
     * speed up image rectification in consecutive stages
     */
    void precomputeRectifiedPoints();

    /**
     * Calculate depth and publish it
     *
     * @see synthesizePointCloudFromVoxelGrid, accumulatePointcloud,
     * publishVoxelGrid, publishDepthmap
     */
    void synthesizeAndPublishDepth();

    /**
     * Build point cloud from voxel grid maximizing the desired focus measure
     *
     * Calls the method defined by type_focus_measure_, @see
     * synthesizePointCloudFromVoxelGridGradMag,
     * synthesizePointCloudFromVoxelGridContrast,
     * synthesizePointCloudFromVoxelGridLinf.
     * The confidence map is normalized (min/max), then it is adaptively
     * thresholded and finally it is median filtered.
     *
     *     ROS params:
     *         type_focus_measure) type of focus measure to use.
     *
     * @param depth [output] depth map
     * @param confidence [output] confidence of the depth
     */
    void synthesizePointCloudFromVoxelGrid(cv::Mat &depth, cv::Mat &confidence);

    /**
     * Build point cloud from voxel grid using Linf norm on optical ray
     *
     * For each pixel, the depth is chosen as the maximum along the optimal ray
     * of the number of votes in the DSI. The confidence is represented by the
     * number of votes.
     *
     * @param depth_cell_indices [output] discrete cell index in the depth axis
     * corresponding to the maxima
     * @param confidence [output] number of votes in the voxel where the maxima
     * is found
     */
    void synthesizePointCloudFromVoxelGridLinf(cv::Mat &depth_cell_indices,
                                               cv::Mat &confidence);

    /**
     * Build point cloud from voxel grid maximizing the contrast on the depth
     * plane
     *
     * For each pixel, the depth is chosen as the maximum along the optimal ray
     * of the gradient magnitude in the DSI. The confidence is represented by
     * the magnitude of the gradient. The 2d gradient is calculated for each
     * depth plane, and the idea is to maximize this focus measure.
     *
     *     ROS params:
     *         half_patchsize) size of the gaussian filter to estimate contrast
     *
     * @param depth_cell_indices [output] discrete cell index in the depth axis
     * corresponding to the maxima
     * @param confidence [output] contrast value in the voxel where the maxima
     * is found
     */
    void synthesizePointCloudFromVoxelGridContrast(cv::Mat &depth_cell_indices,
                                                   cv::Mat &confidence);

    /**
     * Build point cloud from voxel grid maximizing the gradient magnitude on
     * the depth plane
     *
     * For each pixel, the depth is chosen as the maximum along the optimal ray
     * of the gradient magnitude in the DSI. The confidence is represented by
     * the magnitude of the gradient. The 2d gradient is calculated for each
     * depth plane, and the idea is to maximize this focus measure.
     *
     *     ROS params:
     *         half_patchsize) size of the sobel filter to compute algebraic
     *              derivatives
     *
     * @param depth_cell_indices [output] discrete cell index in the depth axis
     * corresponding to the maxima
     * @param confidence [output] magnitude gradient in the voxel where the
     * maxima is found
     */
    void synthesizePointCloudFromVoxelGridGradMag(cv::Mat &depth_cell_indices,
                                                  cv::Mat &confidence);

    /**
     * Builds point cloud from depth map and pose, and publish it
     *
     * Each pixel not masked out from mask is backprojected at the distance
     * provided in depth. The point cloud is filtered with a radius filter (each
     * point is not considered an outlier iff it is at a distance smaller than
     * radius_search_ from min_num_neighbors_ other points).
     * Topic: "pointcloud"
     *
     *     ROS params:
     *         radius_search) radius filter radius size
     *         min_num_neighbors) minimum number of neighbors for radius filter
     *
     * @param depth depth map associating each pixel to a depth value
     * @param mask pixels to be considered
     * @param R_world_ref orientation
     * @param t_world_ref position
     * @param timestamp timestamp of the message to publish
     */
    void accumulatePointcloud(const cv::Mat &depth, const cv::Mat &mask,
                              const cv::Mat &confidence,
                              const Eigen::Matrix3d R_world_ref,
                              const Eigen::Vector3d t_world_ref,
                              const ros::Time &timestamp);

    /**
     * Publishes voxel grid relative to pose
     *
     * Topic: "voxel_grid"
     *
     * @param T_w_ref camera pose
     */
    void publishVoxelGrid(
        const evo_utils::geometry::Transformation &T_w_ref) const;

    /**
     * Publishes depth map
     *
     * Normalizes (minmax) depth map and apply color map before publishing it
     *
     * Remark: it publishes only if there are subscribers
     *
     * Topic: /dvs_mapping/depthmap
     *
     * @param depth Depth associated to each pixel
     * @param mask Pixels to be considered
     */
    void publishDepthmap(const cv::Mat &depth, const cv::Mat &mask);

    /**
     * Publishes global map, after updating it (pc_global_) adding the local
     * point cloud (pc_) under the "pointcloud_global" topic.
     *
     * Remark: it publishes (and updates the pc_global_ point cloud) only if
     * there is at least one subscriber to the topic "pointcloud_global".
     *
     *     ROS params:
     *         voxel_filter_leaf_size) voxel filter granularity
     */
    void publishGlobalMap();

    /**
     * Removes events to keep an history (already processed) of events at most
     * long DEPTH_DEFOCUS_EVENT_HISTORY_SIZE.
     */
    void clearEventQueue();

    /**
     * Setup operations to be performed after dvs_cam_ has been loaded
     *
     *     ROS params:
     *          fov_virtual_camera_deg) fov virtual camera (voxel grid)
     *          virtual_width) voxel grid width
     *          virtual_height) voxel grid height
     */
    void postCameraLoaded();

    /**
     * Retrieves vote counter of a specific (x,y,z) location of the grid
     *
     * This is a util function used for clarity
     *
     * @param grid Grid containing the vote counters
     * @param x x coordinate
     * @param y y coordinate
     * @param z z coordinate
     */
    inline const Vote &voxelGridAt(const std::vector<Vote> &grid, int x, int y,
                                   size_t z) const {
        return grid[x + virtual_width_ * (y + z * virtual_height_)];
    }

    /**
     * Retrieve vote counter of a specific (x,y,z) location of the grid
     *
     * This is a util function used for clarity
     *
     * @param grid Grid containing the vote counters
     * @param x x coordinate
     * @param y y coordinate
     * @param z z coordinate
     */
    inline Vote &voxelGridAt(std::vector<Vote> &grid, int x, int y, size_t z) {
        return grid[x + virtual_width_ * (y + z * virtual_height_)];
    }

    /**
     * Performs map update
     *
     * @see resetVoxelGrid, processEventQueue, synthesizeAndPublishDepth,
     * publishGlobalMap
     */
    void update();

    /** ROS */
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;

    ros::Publisher pub_pc_;          ///< @see accumulatePointcloud;
                                     ///< topic: "pointcloud"
    ros::Publisher pub_pc_global_;   ///< @see publishGlobalMap;
                                     ///< topic: "pointcloud_global"
    ros::Publisher pub_voxel_grid_;  ///< @see publishVoxelGrid callback;
                                     ///< topic: "voxel_grid"

    image_transport::Publisher
        pub_depth_map_;  ///< @see publishDepthMap
                         ///< topic: /dvs_mapping/depthmap

    ros::Subscriber event_sub_;        ///< @see processEventArray
                                       ///< topic: "events"
    ros::Subscriber tf_sub_;           ///< @see tfCallback
                                       ///< topic: "tf"
    ros::Subscriber remote_key_;       ///< @see remoteKeyCallback
                                       ///< topic "remote_key"
    ros::Subscriber camera_info_sub_;  ///< @see cameraInfoCallback
    ros::Subscriber copilot_sub_;      ///< @see copilotCallback

    MapperState state_;  ///< state of the mapper, @see MapperState

    std::string world_frame_id_;  ///< The root frame id of the tf system
    std::string frame_id_;        ///< currently used frame id
    std::string
        regular_frame_id_;  ///< frame id to use during regular operation
    std::string bootstrap_frame_id_;  ///< frame id to use during bootstrapping
    std::shared_ptr<tf::Transformer> tf_;

    evo_utils::geometry::Transformation T_ref_w_;  ///< camera pose
    /** Camera variables */
    image_geometry::PinholeCameraModel
        dvs_cam_;  ///< dvs sensor model retrieved from configuration file
    int width_;    ///< camera width
    int height_;   ///< camera height

    evo_utils::camera::PinholeCamera
        virtual_cam_;  ///< To extend the functionalities of PinholeCameraModel
    int virtual_width_;   ///< virtual width
    int virtual_height_;  ///< virtual height

    Eigen::Matrix3f K_;  ///< intrinsic matrix

    // Ref voxel grid parameters
    /** Voxel grid variables */
    std::vector<Vote> ref_voxel_grid_;  ///< Voxel grid vote counters
    cv::Mat confidence_mask_;           ///< Confidence associated to each voxel
    float voxel_filter_leaf_size_;      ///< Size of the cubic voxel;
                                        ///< ROS param: "voxel_filter_leaf_size"
    size_t
        events_to_recreate_kf_;  ///< Maximum number of events considered to
                                 ///< center the voxel grid at a specific
                                 ///< pose. ROS param: "events_to_recreate_kf"
                                 ///< @see resetVoxelGrid
    int type_focus_measure_;     ///< @see synthesizePointCloudFromVoxelGrid

    Eigen::Matrix2Xf
        precomputed_rectified_points_;  ///< @see precomputeRectifiedPoints

    /** Process events */
    size_t current_event_;         ///< next event to process
                                   ///< @see processEventQueue
    size_t newest_tracked_event_;  ///< first event for which the position is
                                   ///< unknown, @see tfCallback
    size_t last_kf_update_event_;  ///< last event at which the map has been
                                   ///< updated, @see remoteKeyCallback
    std::deque<dvs_msgs::Event> event_queue_;

    PointCloud::Ptr pc_;         ///< Local map
    PointCloud::Ptr pc_global_;  ///< Global map, @see publishGlobalMap

    /** Depth space */
    evo_utils::geometry::Depth min_depth_;  ///< Lower bound in depth range
                                            ///< ROS param: "min_depth"
    evo_utils::geometry::Depth max_depth_;  ///< Upper bound in depth range
                                            ///< ROS param: "max_depth"
    size_t num_depth_cells_;                ///< Number of discrete depth cells
                                            ///< ROS param: "num_depth_cells"
    InverseDepthVector
        depths_vec_;  ///< precomputed vector of num_depth_cells_ inverse depths
    //  LinearDepthVector depths_vec_;
    std::vector<float> raw_depths_vec_;

    float radius_search_;     ///< radius filter: patch radius;
                              ///< ROS param: "radius_search"
    int min_num_neighbors_;   ///< radius filter: min number of neighbors within
                              ///< patch to be considered inlier;
                              ///< ROS param: "min_num_neighbors"
    int median_filter_size_;  ///< size of the median filter @see
                              ///< evo_utils::camera::huangMedianFilter;
                              ///< ROS param: "median_filter_size"

    int adaptive_threshold_kernel_size_;  ///< size of the gaussian kernel used
                                          ///< to compute the adaptive threshold
                                          ///< ROS param:
                                          ///< "adaptive_threshold_kernel_size"
    int adaptive_threshold_c_;  ///< adaptive threshold constant, sum to the
                                ///< weighted average of the neighbors pixel
                                ///< values
                                ///< ROS param: "adaptive_threshold_c"

    bool auto_trigger_;  ///< whether to start on first pose received
};

}  // namespace depth_from_defocus
