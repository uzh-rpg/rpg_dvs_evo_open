#include "motion_correction.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace motion_correction {

void initWarp(cv::Mat& warp, const WarpUpdateParams& params) {
    // Downsample the motion parameters to the lowest pyramid level
    for (size_t lvl = 0; lvl < params.num_pyramid_levels; ++lvl) {
        if (warp.size() == cv::Size(3, 3)) {
            warp.at<float>(0, 2) /= 2.0;
            warp.at<float>(1, 2) /= 2.0;
            warp.at<float>(2, 0) *= 2.0;
            warp.at<float>(2, 1) *= 2.0;
        } else {
            warp.at<float>(0, 2) /= 2.0;
            warp.at<float>(1, 2) /= 2.0;
        }
    }
}

void updateWarp(cv::Mat& warp, const cv::Mat& img0, const cv::Mat& img1,
                const WarpUpdateParams& params) {
    // Initialize with identity or previous warp
    initWarp(warp, params);

    try {
        static cv::Mat img0_8u, img1_8u;
        resetMat(img0_8u, params.sensor_size, CV_8U);
        resetMat(img1_8u, params.sensor_size, CV_8U);

        cv::normalize(img0, img0_8u, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::normalize(img1, img1_8u, 0, 255, cv::NORM_MINMAX, CV_8U);

        std::vector<cv::Mat> pyramid0, pyramid1;
        cv::buildOpticalFlowPyramid(img0_8u, pyramid0, cv::Size(3, 3),
                                    params.num_pyramid_levels, false);
        cv::buildOpticalFlowPyramid(img1_8u, pyramid1, cv::Size(3, 3),
                                    params.num_pyramid_levels, false);
        CHECK_EQ(pyramid0.size(), pyramid1.size());
        VLOG(30) << pyramid0.size();
        VLOG(30) << "Init: " << warp;
        for (size_t lvlz = 0; lvlz < pyramid0.size(); lvlz++) {
            size_t lvl = pyramid0.size() - 1 - lvlz;
            VLOG(30) << "Level: " << lvl;
            VLOG(30) << "Size: " << pyramid0[lvl].size();

            cv::findTransformECC(pyramid0[lvl], pyramid1[lvl], warp,
                                 params.warp_mode, params.criteria);

            VLOG(30) << "After findTransformECC: " << warp;

            // Upsample the warp to prepare for the next level
            if (lvl >= 1) {
                VLOG(30) << "Upsampling the warp";
                if (params.warp_mode == cv::MOTION_HOMOGRAPHY) {
                    warp.at<float>(0, 2) *= 2.0;
                    warp.at<float>(1, 2) *= 2.0;
                    warp.at<float>(2, 0) /= 2.0;
                    warp.at<float>(2, 1) /= 2.0;
                } else {
                    warp.at<float>(0, 2) *= 2.0;
                    warp.at<float>(1, 2) *= 2.0;
                }
            }
        }
    } catch (cv::Exception& e) {
        LOG(WARNING) << e.err;
        if (params.warp_mode == cv::MOTION_HOMOGRAPHY) {
            warp = cv::Mat::eye(3, 3, CV_32F);
        } else {
            warp = cv::Mat::eye(2, 3, CV_32F);
        }
    }
}

cv::Mat computeFlowFromWarp(const cv::Mat& warp, double dt,
                            cv::Size sensor_size,
                            std::vector<cv::Point2f> rectified_points) {
    CHECK(warp.size() == cv::Size(3, 2) || warp.size() == cv::Size(3, 3));
    CHECK_EQ(warp.type(), CV_32F);

    static cv::Mat flow_field;
    resetMat(flow_field, sensor_size, CV_32FC2);

    const bool is_homography = warp.size() == cv::Size(3, 3);

    // pre-initialize matrices pts_begin_2x1 and pts_begin_homogeneous_3x1
    // which contain the rectified undistorted pixel locations
    // (normal and homogeneous coordinates)
    static cv::Mat pts_begin_2x1;
    static cv::Mat pts_begin_homogeneous_3x1;
    static bool pts_begin_initialized = false;
    if (!pts_begin_initialized) {
        pts_begin_2x1 = cv::Mat::zeros(sensor_size, CV_32FC2);
        pts_begin_homogeneous_3x1 = cv::Mat::zeros(sensor_size, CV_32FC3);
        for (size_t y = 0; y < sensor_size.height; ++y) {
            for (size_t x = 0; x < sensor_size.width; ++x) {
                const cv::Point2f& p =
                    rectified_points[x + y * sensor_size.width];
                pts_begin_2x1.at<cv::Vec2f>(y, x) = cv::Vec2f(p.x, p.y);
                pts_begin_homogeneous_3x1.at<cv::Vec3f>(y, x) =
                    cv::Vec3f(p.x, p.y, 1.f);
            }
        }
        pts_begin_initialized = true;
        VLOG(20) << "Initialized pts_begin_2x1 and pts_begin_homogeneous_3x1";
    }

    cv::Mat pts_end;
    cv::Mat* pts_begin;
    pts_begin = (is_homography) ? &pts_begin_2x1 : &pts_begin_homogeneous_3x1;

    // @TODO
    // We use numerical derivatives here for simplicity although there is a
    // closed form solution.
    if (is_homography) {
        cv::perspectiveTransform(*pts_begin, pts_end, warp);
    } else  // affine, euclidean, or translation
    {
        cv::transform(*pts_begin, pts_end, warp);
    }

    for (size_t y = 0; y < sensor_size.height; ++y) {
        for (size_t x = 0; x < sensor_size.width; ++x) {
            cv::Vec2f& flow = flow_field.at<cv::Vec2f>(y, x);
            if (is_homography) {
                flow = (pts_end.at<cv::Vec2f>(y, x) -
                        pts_begin->at<cv::Vec2f>(y, x)) /
                       dt;
            } else {
                flow[0] = (pts_end.at<cv::Vec2f>(y, x)[0] -
                           pts_begin->at<cv::Vec3f>(y, x)[0]) /
                          dt;
                flow[1] = (pts_end.at<cv::Vec2f>(y, x)[1] -
                           pts_begin->at<cv::Vec3f>(y, x)[1]) /
                          dt;
            }
        }
    }

    return flow_field;
}

void drawEventsUndistorted(EventArray::iterator ev_first,
                           EventArray::iterator ev_last, cv::Mat& out,
                           cv::Size sensor_size,
                           const std::vector<cv::Point2f>& rectified_points,
                           const bool use_polarity) {
    // Draw Events
    auto draw = [](float& p, const float val) { p = p + val; };

    out = cv::Scalar(0);

    float w0, w1, w2, w3;

    cv::Point2i tl;

    for (auto e = ev_first; e != ev_last; ++e) {
        const cv::Point2f& p =
            rectified_points[e->x + e->y * sensor_size.width];

        tl.x = int_floor(p.x);
        tl.y = int_floor(p.y);
        if (tl.x >= 0 && tl.x < sensor_size.width - 1 && tl.y >= 0 &&
            tl.y < sensor_size.height - 1) {
            const float fx = p.x - tl.x, fy = p.y - tl.y;
            w0 = (1.f - fx) * (1.f - fy);
            w1 = (fx) * (1.f - fy);
            w2 = (1.f - fx) * (fy);
            w3 = (fx) * (fy);

            float pol;

            if (use_polarity) {
                pol = ((e->polarity) ? 1.f : -1.f);
            } else {
                pol = 1.f;
            }

            draw(out.at<float>(tl.y, tl.x), pol * w0);
            draw(out.at<float>(tl.y, tl.x + 1), pol * w1);
            draw(out.at<float>(tl.y + 1, tl.x), pol * w2);
            draw(out.at<float>(tl.y + 1, tl.x + 1), pol * w3);
        }
    }
}

void drawEventsMotionCorrectedOpticalFlow(
    EventArray::iterator ev_first, EventArray::iterator ev_last,
    const cv::Mat& flow_field, cv::Mat& out, cv::Size sensor_size,
    const std::vector<cv::Point2f>& rectified_points, const bool use_polarity) {
    CHECK_EQ(flow_field.type(), CV_32FC2);

    // Draw Events
    auto draw = [](float& p, const float val) { p = p + val; };

    out = cv::Scalar(0);

    float w0, w1, w2, w3;

    cv::Point2i tl;
    cv::Point2f p_corr;

    const ros::Time t_begin = ev_first->ts;
    const ros::Time t_end = ev_last->ts;
    for (auto e = ev_first; e != ev_last; ++e) {
        const cv::Point2f& p =
            rectified_points[e->x + e->y * sensor_size.width];
        const cv::Vec2f& flow = flow_field.at<cv::Vec2f>(p.y, p.x);
        const double dt = (t_end - e->ts).toSec();
        p_corr.x = p.x + dt * flow[0];
        p_corr.y = p.y + dt * flow[1];

        tl.x = int_floor(p_corr.x);
        tl.y = int_floor(p_corr.y);
        if (tl.x >= 0 && tl.x < sensor_size.width - 1 && tl.y >= 0 &&
            tl.y < sensor_size.height - 1) {
            const float fx = p_corr.x - tl.x, fy = p_corr.y - tl.y;
            w0 = (1.f - fx) * (1.f - fy);
            w1 = (fx) * (1.f - fy);
            w2 = (1.f - fx) * (fy);
            w3 = (fx) * (fy);

            float pol;

            if (use_polarity) {
                pol = ((e->polarity) ? 1.f : -1.f);
            } else {
                pol = 1.f;
            }

            draw(out.at<float>(tl.y, tl.x), pol * w0);
            draw(out.at<float>(tl.y, tl.x + 1), pol * w1);
            draw(out.at<float>(tl.y + 1, tl.x), pol * w2);
            draw(out.at<float>(tl.y + 1, tl.x + 1), pol * w3);
        }
    }
}
}  // namespace motion_correction
