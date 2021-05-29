#ifndef LK_SE3_H
#define LK_SE3_H

#include <dvs_msgs/Event.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/common/transforms.h>

#include <Eigen/StdVector>
#include <deque>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

/**
 * Generic LK tracker based on SE3
 */
class LKSE3 {
    typedef Eigen::Matrix<float, 6, 6> Matrix6;
    typedef Eigen::Matrix<float, 8, 8> Matrix8;
    typedef Eigen::Matrix<float, 6, 1> Vector6;
    typedef Eigen::Matrix<float, 8, 1> Vector8;

   public:
    /**
     * Util structure to update the transformation, used to define the generic
     * jacobian and store the bearing vector and the intensity value of the
     * tracked frame
     */
    struct Keypoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3f P;  ///< bearing vector
        float pixel_value;  ///< corresponding intensity value
        Vector8 J;          ///< Jacobian of the trasformation to fit
        Matrix6 JJt;        ///< J dot J^T

        Keypoint(Eigen::Vector3f _P, float _pixel_value, Vector8 _J,
                 Matrix6 _JJt)
            : P(_P), pixel_value(_pixel_value), J(_J), JJt(_JJt) {}
    };

    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef std::deque<dvs_msgs::Event> EventQueue;

    typedef Sophus::SE3f SE3;

   protected:
    size_t batch_size_;      ///< number of keypoints processed together (batch)
    size_t max_iterations_;  ///< maximum number of iterations during the
                             ///< optimization step, @see updateTransformation,
                             ///< drawEvents, drawEventsNN
    size_t pyramid_levels_;  ///< number of levels in the image pyramid
                             ///< TODO: This could also be done relying on
                             ///< @see pyr_new_.size()
    size_t batches_;         ///< number of batches
                             ///< @see keypoints_, batches_size_

    float weight_scale_trans_;  ///< weight update due to translational residual
    float weight_scale_rot_;    ///< weight update due to rotational residual

    int map_blur_;        ///< Blur value of map projected onto the img_ref_
    float depth_median_;  ///< median depth in current scene (reference frame)

    std::vector<Keypoint, Eigen::aligned_allocator<Keypoint> >
        keypoints_;  ///< Keypoints in reference frame @see LKSE3::Keypoint
    int n_visible_;  ///< Number of keypoints of the reference frame visible in
                     ///< the current frame
    float kf_visibility_;  ///< Percentage of keypoints of the reference frame
                           ///< visible in the current frame
    PointCloud::Ptr map_;  ///< Map (built in the reference keyframe c_ref_)
    PointCloud::Ptr map_local_;  ///< Submap visible in the current frame

    cv::Mat depth_ref_;             ///< Depth map in the reference keyframe
    cv::Mat ref_img_;               ///< Image of the reference keyframe
    cv::Mat new_img_;               ///< Current image
    std::vector<cv::Mat> pyr_new_;  ///< Image pyramid of the new image

    image_geometry::PinholeCameraModel c_;      // camera at the current frame
    image_geometry::PinholeCameraModel c_ref_;  // camera at the reference frame
    float fx_;       ///< image horizontal focal length
    float fy_;       ///< image vertical focal length
    float cx_;       ///< image horizontal principal point
    float cy_;       ///< image vertical principal point
    size_t width_;   ///< image width
    size_t height_;  ///< image height
    cv::Rect rect_;  ///< image container, width x height

    // Poses
    Eigen::Affine3f
        T_world_kf_;  ///< Trasformation from keyframe to world frame
    Eigen::Affine3f
        T_kf_ref_;  ///< Trasformation from reference frame to keyframe
    Eigen::Affine3f
        T_ref_cam_;  ///< Trasformation from current frame to reference frame
    Eigen::Affine3f
        T_cur_ref_;  ///< Trasformation from reference frame to current frame

    Eigen::VectorXf x_ = Vector6::Zero();  ///< Twist vector <-> T_cur_ref

    /**
     * Creates a new reference frame
     *
     * Project map onto the current reference frame (img_ref_), build local map
     * (map_local_), evaluate quality of new reference frame (kf_visibility_,
     * n_visible_), compute median depth of the scene (depth_median_) and
     * precompute the data useful for the optimization in the tracking step.
     *
     * @see precomputeReferenceFrame
     */
    void projectMap();
    /**
     * Computes keypoints on the reference image ref_img_ to perform the KLT,
     * with the Jacobian evaluated at each point of the transformation to fit
     *
     * @see LKSE3::Keypoint
     */
    void precomputeReferenceFrame();

    /**
     * Draws events given in rectified image (when visible after rectification)
     * with bilinear interpolation
     *
     * @param ev_first iterator to first event in events container
     * @param ev_last iterator representing end of events container (not last
     * event!, e.g., vector.end())
     * @param out [output] image containing events drawed within
     *
     * @see rect_, drawEventsNN
     */
    void drawEvents(EventQueue::iterator ev_first, EventQueue::iterator ev_last,
                    cv::Mat &out);

    /**
     * Draws events given in rectified image (when visible after rectification)
     *
     * @param ev_first iterator to first event in events container
     * @param ev_last iterator representing end of events container (not last
     * event!, e.g., vector.end())
     * @param out [output] image containing events drawed within
     *
     * @see rect_, drawEvents
     */
    void drawEventsNN(EventQueue::iterator ev_first,
                      EventQueue::iterator ev_last, cv::Mat &out);

    /**
     * Updates transformation: one pass of stochastic gradient descent
     *
     * Remark: keypoints_ are already shuffled in precomputeReferenceFrame
     *
     * @param offset first keypoint to consider
     * @param N number of keypoints to consider
     * @param pyr_lvl level of the image pyramid
     */
    void updateTransformation(const int offset, const int N, size_t pyr_lvl);

    /**
     * Tracks frame updating the transformation at each pyramid_levels_ and for
     * a maximum of max_iterations_
     *
     * @see updateTransformation
     */
    void trackFrame();
};

#endif  // LK_SE3_H
