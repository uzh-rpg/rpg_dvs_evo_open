#pragma once

#ifndef INTERPOLATION_HPP
#define INTERPOLATION_HPP

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/core/core.hpp>

namespace evo_utils::interpolate {

/**
 * Bilinear interpolation
 *
 * If all the four neighbors are available, computes the bilinear interpolation
 * of their associated values in img. Otherwise, returns -1.f
 *
 * @param img values associated to the pixels
 * @param w image width
 * @param h image height
 * @param x horizontal coordinate (continuous space)
 * @param y vertical coordinate (continuous space)
 *
 * @return bilinear interpolation of the four neighbors value, -1.f is one
 * neighbors is out of bound
 */
float bilinear(const float* img, const size_t w, const size_t h, const float x,
               const float y) {
    if (x < 0 || y < 0) return -1.f;

    size_t px = (size_t)x, py = (size_t)y;

    if (px + 1 >= w || py + 1 >= h) return -1.f;

    // Load Pixel Values
    const float* addr = img + px + py * w;
    const float p1 = addr[0], p2 = addr[1], p3 = addr[w], p4 = addr[w + 1];

    // Compute Weights
    float fx = x - px, fy = y - py, fx1 = 1.0f - fx, fy1 = 1.0f - fy;

    float w1 = fx1 * fy1, w2 = fx * fy1, w3 = fx1 * fy, w4 = fx * fy;

    return p1 * w1 + p2 * w2 + p3 * w3 + p4 * w4;
}

/**
 * Bilinear interpolation
 *
 * If all the four neighbors are available, computes the bilinear interpolation
 * of their associated values in img. Otherwise, returns -1.f
 *
 * @param img cv::Mat image
 * @param x horizontal coordinate (continuous space)
 * @param y vertical coordinate (continuous space)
 *
 * @return bilinear interpolation of the four neighbors value, -1.f is one
 * neighbors is out of bound
 *
 * @see bilinear
 */
inline float bilinear(const cv::Mat& img, const float x, const float y) {
    return bilinear((float*)img.data, (size_t)img.cols, (size_t)img.rows, x, y);
}

/**
 * Nearest neighbourhood interpolation
 *
 * If the nearest neighbour is outside of the boundaries, returns -1.f.
 * Otherwise, returns its value.
 *
 * @param img values associated to the pixels
 * @param w image width
 * @param h image height
 * @param x horizontal coordinate (continuous space)
 * @param y vertical coordinate (continuous space)
 *
 * @return value associated to nearest neighbour or -1.f, if it is outside of
 * the boundaries of the image
 */
float nn(const float* img, const size_t w, const size_t h, const float x,
         const float y) {
    if (x < 0 || y < 0) return -1;

    size_t px = static_cast<size_t>(x + .5f), py = static_cast<size_t>(y + .5f);

    if (px >= w || py >= h) return -1;

    return img[px + py * w];
}

/**
 * Distribute val around (x,y) so that the bilinear interpolation at (x,y) is
 * increased by val
 *
 * @param img image to which the value is added
 * @param x horizontal coordinate of the increment
 * @param y vertical coordinate of the increment
 * @param val value increment
 */
void draw(cv::Mat& img, const float x, const float y, const float val) {
    const int w = img.cols, h = img.rows;

    if (x < 0 || y < 0) return;

    const int px = (int)x, py = (int)y;

    if (px + 1 >= w || py + 1 >= h) return;

    float* data = (float*)img.data;

    const float fx = x - px, fy = y - py;

    data[px + py * w] += val * (1.f - fx) * (1.f - fy);
    data[px + 1 + py * w] += val * (fx) * (1.f - fy);
    data[px + (py + 1) * w] += val * (1.f - fx) * (fy);
    data[px + 1 + (py + 1) * w] += val * (fx) * (fy);
}

}  // namespace interpolate

#endif  // INTERPOLATION_HPP
