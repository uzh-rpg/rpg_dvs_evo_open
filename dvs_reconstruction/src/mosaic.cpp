#include "dvs_reconstruction/mosaic.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

// #define MOSAIC_PERF

#ifdef MOSAIC_PERF
#include "evo_utils/main.hpp">
#endif

#include "dvs_reconstruction/poisson_solver/laplace.h"
#include "evo_utils/interpolation.hpp"
#include "rpg_common_ros/params_helper.hpp"

Mosaic::Mosaic(const float sigma_m, const float init_cov, int window_size,
               int map_blur, tf::TransformListener &tf, ros::NodeHandle &nh,
               ros::NodeHandle &nhp)
    : nh_(nh),
      nhp_(nhp),
      tf_(tf),
      it_(nh),
      R_(sigma_m),
      init_cov_(init_cov),
      window_size_(window_size),
      map_blur_(map_blur) {
    frame_id_ = rpg_common_ros::param(nh_, "dvs_frame_id", std::string("dvs"));
    world_frame_id_ =
        rpg_common_ros::param(nh_, "world_frame_id", std::string("world"));
}

void Mosaic::setCamModel(image_geometry::PinholeCameraModel &c) {
    static const float s = nhp_.param("super_resolution_factor", 1.);
    c_ = c;
    K_ = tf::Matrix3x3(c.fx(), 0., c.cx(), 0., c.fy(), c.cy(), 0., 0., 1.);
    KInv_ = K_.inverse();
    w_ = c.fullResolution().width;
    h_ = c.fullResolution().height;

    K_virt_ = tf::Matrix3x3(s * c.fx(), 0., s * c.cx(), 0., s * c.fy(),
                            s * c.cy(), 0., 0., 1.);
    KInv_virt_ = K_virt_.inverse();
    w_virt_ = s * c.fullResolution().width;
    h_virt_ = s * c.fullResolution().height;

    precomputeRectificationMap();
}

void Mosaic::compute(const std::vector<dvs_msgs::Event> &evs,
                     const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &map) {
    int pose_update_step = 500, map_update_step = 10000;

    if (evs.size() == 0) return;

    ros::Time t1 = ts_ref_ = evs.back().ts;

    cv::Mat img(h_virt_, w_virt_, CV_32F, cv::Scalar(0.));
    cv::Mat depth(h_, w_, CV_32F, cv::Scalar(0.));

    tf::StampedTransform T;

    // Convert Map
    map_.clear();
    for (size_t i = 0; i != map->size(); ++i) {
        const pcl::PointXYZ &p = map->points[i];
        map_.push_back(tf::Point(p.x, p.y, p.z));
    }
    tf::StampedTransform T_map;
    tf_.waitForTransform(world_frame_id_, frame_id_, ts_ref_,
                         ros::Duration(0.1));
    tf_.lookupTransform(world_frame_id_, frame_id_, ts_ref_, T_map);
    reprojectDepthmap(T_map, depthmap, true);
    // publishDepthmap();

    grad = cv::Mat_<cv::Vec2f>(h_virt_, w_virt_, cv::Vec2f(0., 0.));
    cov = cv::Mat_<cv::Vec4f>(h_virt_, w_virt_,
                              cv::Vec4f(init_cov_, 0., 0., init_cov_));

#ifdef MOSAIC_PERF
    TIMER_START(t1);
#endif

    for (size_t i = 0; i != evs.size(); ++i) {
        int idx = evs.size() - 1 - i;
        const dvs_msgs::Event &e = evs[idx];

        if (i % map_update_step == 0) {
            tf::StampedTransform T_map;
            tf_.lookupTransform(world_frame_id_, frame_id_, evs[idx].ts, T_map);
            reprojectDepthmap(T_map, depth);
        }

        if (i % pose_update_step == 0) {
            tf_.lookupTransform(frame_id_, ts_ref_, frame_id_, evs[idx].ts,
                                world_frame_id_, T);
        }

        if (i != 0 && ((i + 1) % window_size_ == 0)) {
            update(img, e.ts, t1);
            img = cv::Scalar(0.);
            t1 = e.ts;
        }

        cv::Point2d px = rectification_map_[e.x + w_ * e.y];

        if (px.x < 0) continue;

        float z = depth.at<float>(px);

        // Project Event on Map
        tf::Point p(px.x, px.y, 1.);
        p = T * (z * (KInv_ * p));

        if (p.z() < 0.01f) continue;

        p = K_virt_ * (p / p.z());

        float pol = e.polarity ? +1 : -1;
        evo_utils::interpolate::draw(img, p.x(), p.y(), pol);
    }

#ifdef MOSAIC_PERF
    TIMER_STOP(t1, t2, duration);
    LOG(INFO) << "Duration: " << duration << " [ms]";
#endif

    publishImageReconstruction();
    publishDepthmap(depthmap);
}

void Mosaic::publishImageReconstruction() {
    static image_transport::Publisher img_pub =
        it_.advertise("dvs_reconstruction/image", 1);

    if (img_pub.getNumSubscribers() > 0) {
        cv::Mat img = getReconstructedImage();
        cv::convertScaleAbs(img, img, 255);

        std_msgs::Header header;
        header.stamp = ts_ref_;

        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(header, "mono8", img).toImageMsg();
        img_pub.publish(msg);
    }
}

void Mosaic::publishDepthmap(const cv::Mat &depth) {
    static image_transport::Publisher depth_pub =
        it_.advertise("dvs_reconstruction/depthmap", 1);

    if (depth_pub.getNumSubscribers() > 0) {
        cv::Mat img;
        // cv::normalize(depthmap, img, 0, 255, cv::NORM_MINMAX);
        cv::normalize(depth, img, 0, 255, cv::NORM_MINMAX);
        cv::convertScaleAbs(img, img);
        cv::applyColorMap(img, img, cv::COLORMAP_JET);
        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        depth_pub.publish(msg);
    }
}

void Mosaic::reprojectDepthmap(const tf::Transform &T, cv::Mat &_depthmap,
                               bool virtual_cam) {
    const tf::Matrix3x3 &K = virtual_cam ? K_virt_ : K_;
    const size_t h_orig = virtual_cam ? h_virt_ : h_,
                 w_orig = virtual_cam ? w_virt_ : w_,
                 h = h_orig + map_blur_ - 1, w = w_orig + map_blur_ - 1;

    cv::Mat weights(h, w, CV_32F, cv::Scalar(0.));
    cv::Mat depth(h, w, CV_32F, cv::Scalar(0.));

    cv::Mat kern, patch;

    kern = cv::getGaussianKernel(map_blur_, 0., CV_32F);
    kern = kern * kern.t();
    tf::Transform TInv = T.inverse();

    static std::vector<float> z_values;
    z_values.clear();

    for (size_t i = 0; i != map_.size(); ++i) {
        tf::Vector3 p = TInv * map_[i];
        float z = p.z();
        z_values.push_back(z);

        if (z < 0.01f) continue;

        p = K * (p / z);
        if (p.x() < map_blur_ / 2 || p.y() < map_blur_ / 2) continue;

        size_t x = p.x() + .5f, y = p.y() + .5f;

        if (x >= w_orig || y >= h_orig) continue;

        patch = z * kern;

        cv::Rect dst(x, y, map_blur_, map_blur_);
        depth(dst) += patch;
        weights(dst) += kern;
    }

    depth /= weights;

    // Fill with Median Depth
    std::nth_element(z_values.begin(), z_values.begin() + z_values.size() / 2,
                     z_values.end());
    depth_median_ = z_values[z_values.size() / 2];

    for (size_t y = 0; y != h; ++y)
        for (size_t x = 0; x != w; ++x)
            if (depth.at<float>(y, x) < .1f)
                depth.at<float>(y, x) = depth_median_;

    cv::medianBlur(depth, depth, 3);

    int b2 = map_blur_ / 2;
    _depthmap = depth(cv::Rect(b2, b2, w_orig, h_orig)).clone();

    //  display_image_normalized(_depthmap);
}

cv::Mat Mosaic::getAbsoluteGradient() {
    cv::Mat img(grad.size(), CV_32F);

    for (size_t y = 0; y != h_virt_; ++y)
        for (size_t x = 0; x != w_virt_; ++x)
            img.at<float>(y, x) = cv::norm(grad(y, x));

    cv::normalize(img, img, 0, 1, cv::NORM_MINMAX);

    return img;
}

cv::Mat Mosaic::getReconstructedImage() {
    int w_ = grad.cols, h_ = grad.rows;
    boost::multi_array<double, 2> F(boost::extents[h_][w_]);
    boost::multi_array<double, 2> U;

    // Calculate Second Derivative
    for (int y = 0; y != h_ - 1; ++y)
        for (int x = 0; x != w_ - 1; ++x) {
            float uxx = grad(y, x + 1)[0] - grad(y, x)[0],
                  vxx = grad(y + 1, x)[1] - grad(y, x)[1];
            F[y][x] = uxx + vxx;
        }
    F[h_ - 1][w_ - 1] = 0.;

    // Solve for M_xx + M_yy = F using Poisson solver,
    // with constant intensity boundary conditions
    double boundary = 0.;
    pde::poisolve(U, F, 1., 1., 1., 1., boundary,
                  pde::types::boundary::Dirichlet, false);

    // Copy Multiarray to cv::Mat
    cv::Mat img(h_, w_, CV_32F);
    for (int y = 0; y != h_; ++y)
        for (int x = 0; x != w_; ++x) img.at<float>(y, x) = U[y][x];

    cv::normalize(img, img, 0, 1, cv::NORM_MINMAX);

    return img;
}

void Mosaic::precomputeRectificationMap() {
    for (size_t y = 0; y != h_; ++y)
        for (size_t x = 0; x != w_; ++x) {
            cv::Point2d p(c_.rectifyPoint(cv::Point2d(x, y)));
            cv::Point2i p_i = p;

            if (p_i.x < 0 || p_i.x >= (int)w_ || p_i.y < 0 || p_i.y >= (int)h_)
                rectification_map_.push_back(cv::Point2d(-1, -1));
            else
                rectification_map_.push_back(p);
        }
}

void Mosaic::update(const cv::Mat &new_grad, const ros::Time &t0,
                    const ros::Time &t1) {
    float min_grad = .0001;

    tf::StampedTransform T0, T1;
    tf_.lookupTransform(frame_id_, t0, frame_id_, ts_ref_, world_frame_id_, T0);
    tf_.lookupTransform(frame_id_, t1, frame_id_, ts_ref_, world_frame_id_, T1);

    for (size_t y = 0; y != h_virt_; ++y)
        for (size_t x = 0; x != w_virt_; ++x) {
            const Scalar &z = new_grad.at<float>(y, x);

            if (std::abs(z(0, 0)) < min_grad) continue;

            const float depth = depthmap.at<float>(y, x);

            Gradient &g = grad(y, x);
            Covariance &P = cov(y, x);

            // Velocity
            tf::Point p_ref = depth * (KInv_virt_ * tf::Point(x, y, 1.)),
                      p0 = T0 * p_ref, p1 = T1 * p_ref;

            if (p0.z() < 0.01f || p1.z() < 0.01f) continue;

            p0 = K_virt_ * (p0 / p0.z());
            p1 = K_virt_ * (p1 / p1.z());

            const Velocity v(p0.x() - p1.x(), p0.y() - p1.y());

            // EKF Update Equations
            const cv::Matx12f dhdg = v.t();
            const Scalar h = g.dot(v), nu = z - h, S = dhdg * P * dhdg.t() + R_;
            const cv::Matx21f W = P * dhdg.t() * S.inv();

            // Apply Update
            g += W * nu;
            P -= W * S * W.t();
        }
}
