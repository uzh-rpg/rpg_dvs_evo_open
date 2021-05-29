#ifndef GRADIENT_ESTIMATION_H
#define GRADIENT_ESTIMATION_H

#include <dvs_msgs/Event.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

// #define MOSAIC_PERF ///< uncomment to measure performance

/**
 * Reconstructs image from events using poisson equation solver
 *
 * @see laplace.h
 */
class Mosaic {
   public:
    typedef cv::Vec2f Velocity;
    typedef cv::Vec2f Gradient;
    typedef cv::Matx<float, 2, 2> Covariance;
    typedef cv::Matx<float, 1, 1> Scalar;
    typedef double Time;

    /**
     * Constructor of image reconstructor
     *
     * @param sigma_m sigma_m for the EKF
     * @param init_cov initial guess for the EKF covariance
     * @param window_size window of events between EKF update
     * @param map_blur kernel of map reprojection blur
     * @param tf ros transform listener
     * @param nh node handle
     * @param nhp private node handle
     */
    Mosaic(const float sigma_m, const float init_cov, int window_size,
           int map_blur, tf::TransformListener &tf, ros::NodeHandle &nh,
           ros::NodeHandle &nhp);

    /**
     * Setups camera model, store intrinsic matrix, prepare virtual camera
     * (super-resolution) and pre-compute rectification map.
     *
     *      ROS params:
     *          super_resolution_factor) super resolution of the reconstruction
     *              algorithm
     *
     * @param c pinhole camera model
     *
     * @see precomputeRectificationMap
     */
    void setCamModel(image_geometry::PinholeCameraModel &c);

    /**
     * Processes a batch of events and compute image reconstruction
     *
     *      ROS params:
     *          dvs_frame_id) frame id of the estimated pose
     *          world_frame_id) world frame id
     *
     * @param evs events to process in order to reconstruct the image
     * @param map scene map used to get 3d information
     *
     * @see update, reprojectDepthmap
     */
    void compute(const std::vector<dvs_msgs::Event> &evs,
                 const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &map);

    /**
     * Returns the magnitude of the gradient values stored in grad, normalized
     * (MINMAX) between 0 and 1.
     *
     * @return the magnitude of the normalized (MINMAX) gradient at each pixel
     */
    cv::Mat getAbsoluteGradient();

    /**
     * Calculate second derivative (adjacent differences) and solve for
     * M_xx + M_yy = F using Poisson solver
     *
     * @see pde::poisolve
     */
    cv::Mat getReconstructedImage();

    /**
     * Reproject the map and compute the depth
     *
     * For each point, its depth is distributed in a patch around its projection
     * in the current image accordingly to a gaussian scheme. The total depth is
     * then normalized with the total sum of the weights used in each pixel.
     * Each pixel with a value smaller then 0.1 is filled with the median depth
     * value. Finally, a median filtering is performed.
     *
     * @param T from reference frame of the map to current frame
     * @param _depthmap [output] resulting depth map
     * @param virtual_cam whether to use or not super resolution
     */
    void reprojectDepthmap(const tf::Transform &T, cv::Mat &_depthmap,
                           bool virtual_cam = false);

    cv::Mat_<Gradient> grad;   ///< gradient map (2 channels of float)
    cv::Mat_<Covariance> cov;  ///< gradient covariances (2x2 matrix stored in 4
                               ///< channels of float)
    cv::Mat depthmap;          ///< Dense Depthmap projected in Map Frame
    cv::Mat image;             ///< Reconstructed Image
   protected:
    /* ROS */
    ros::NodeHandle &nh_, &nhp_;
    tf::TransformListener &tf_;
    image_transport::ImageTransport it_;

    Scalar R_;
    float init_cov_;      ///< initial guess for the covariance of the EKF
    size_t window_size_;  ///< window of events between EKF updates
    size_t map_blur_;     ///< gaussian blur kernel size
    float depth_median_;  ///< median scene depth

    image_transport::Publisher pub_img_;    ///< @see publishImageReconstruction
    image_transport::Publisher pub_depth_;  ///< @see publishDepthmap

    ros::Time ts_ref_;  ///< reference timestamp, used to recover the pose and
                        ///< re-construct the image

    std::string frame_id_;  ///< camera transformation frame id, "dvs_frame_id"
    std::string world_frame_id_;  ///< The root of the tf system

    std::vector<cv::Point2d>
        rectification_map_;  ///< @see precomputeRectificationMap

    image_geometry::PinholeCameraModel c_;  ///< camera model
    size_t w_;                              ///< image width
    size_t h_;                              ///< image height
    tf::Matrix3x3 K_;                       ///< intrinsic camera matrix
    tf::Matrix3x3 KInv_;                    ///< inverse of K_

    /* virtual camera -> super resolution s, param: "super_resolution_factor */
    size_t w_virt_;            ///< virtual camera width = s * w_
    size_t h_virt_;            ///< virtual camera height = s * h_
    tf::Matrix3x3 K_virt_;     ///< intrinsic camera matrix of virtual camera
    tf::Matrix3x3 KInv_virt_;  ///< inverse of K_virt_

    std::vector<tf::Vector3> map_;  ///< 3D Point Cloud for Depthmap

    /**
     * Performs the update step of the EKF (update grad, cov) for each pixel
     *
     *      ROS params:
     *          dvs_frame_id) frame id of the estimated pose
     *          world_frame_id) world frame id
     *
     * @param new_grad new gradient of the image
     * @param t0 time of last update
     * @param t1 time of current update
     */
    void update(const cv::Mat &new_grad, const ros::Time &t0,
                const ros::Time &t1);

    /**
     * Computes values for rectification_map_, that maps the pixel values to
     * their corresponding rectified. If outside boundaries (-1, -1) is used.
     */
    void precomputeRectificationMap();

    /**
     * Publishes image reconstruction
     *
     * Topic: "dvs_reconstruction/image"
     *
     * @see getReconstructedImage()
     */
    void publishImageReconstruction();

    /**
     * Publishes depthmap (normalized) with colormap: cv::COLORMAP_JET
     *
     * Topic: "dvs_reconstruction/depthmap"
     *
     * @param depth depthmap to publish
     */
    void publishDepthmap(const cv::Mat &depth);
};

#endif  // GRADIENT_ESTIMATION_H
