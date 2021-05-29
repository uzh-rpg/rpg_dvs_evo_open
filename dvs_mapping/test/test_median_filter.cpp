#include <aslam/common/entrypoint.h>
#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include "evo_utils/geometry.hpp"

using namespace evo_utils::geometry;

class MedianFilterTest : public ::testing::Test {
   protected:
    virtual void SetUp() {
        rng_ = cv::RNG();

        img_ = cv::Mat(6, 6, CV_8U);

        img_.at<uchar>(0, 0) = 3;
        img_.at<uchar>(0, 1) = 1;
        img_.at<uchar>(0, 2) = 2;
        img_.at<uchar>(0, 3) = 9;
        img_.at<uchar>(0, 4) = 8;
        img_.at<uchar>(0, 5) = 3;

        img_.at<uchar>(1, 0) = 1;
        img_.at<uchar>(1, 1) = 4;
        img_.at<uchar>(1, 2) = 5;
        img_.at<uchar>(1, 3) = 2;
        img_.at<uchar>(1, 4) = 0;
        img_.at<uchar>(1, 5) = 0;

        img_.at<uchar>(2, 0) = 9;
        img_.at<uchar>(2, 1) = 8;
        img_.at<uchar>(2, 2) = 1;
        img_.at<uchar>(2, 3) = 2;
        img_.at<uchar>(2, 4) = 5;
        img_.at<uchar>(2, 5) = 5;

        img_.at<uchar>(3, 0) = 1;
        img_.at<uchar>(3, 1) = 5;
        img_.at<uchar>(3, 2) = 8;
        img_.at<uchar>(3, 3) = 1;
        img_.at<uchar>(3, 4) = 8;
        img_.at<uchar>(3, 5) = 8;

        img_.at<uchar>(4, 0) = 5;
        img_.at<uchar>(4, 1) = 4;
        img_.at<uchar>(4, 2) = 3;
        img_.at<uchar>(4, 3) = 2;
        img_.at<uchar>(4, 4) = 1;
        img_.at<uchar>(4, 5) = 0;

        img_.at<uchar>(5, 0) = 9;
        img_.at<uchar>(5, 1) = 9;
        img_.at<uchar>(5, 2) = 8;
        img_.at<uchar>(5, 3) = 7;
        img_.at<uchar>(5, 4) = 6;
        img_.at<uchar>(5, 5) = 5;

        //    print_array(img_, 6, 6);
    }

    void print_array(const cv::Mat& arr, int cols, int rows) {
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                std::cout << (int)arr.at<uchar>(y, x) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    cv::Mat img_;
    cv::RNG rng_;
};

TEST_F(MedianFilterTest, NaiveFilterWithoutMask) {
    cv::Mat mask = cv::Mat::ones(6, 6, CV_8U);
    cv::Mat filtered_img;
    naiveMedianFilter(img_, filtered_img, mask, 3);

    //  print_array(filtered_img, 6, 6);

    // Note: we don't check the boundaries
    EXPECT_EQ(filtered_img.at<uchar>(1, 1), 3);
    EXPECT_EQ(filtered_img.at<uchar>(1, 2), 2);
    EXPECT_EQ(filtered_img.at<uchar>(1, 3), 2);
    EXPECT_EQ(filtered_img.at<uchar>(1, 4), 3);

    EXPECT_EQ(filtered_img.at<uchar>(2, 1), 5);
    EXPECT_EQ(filtered_img.at<uchar>(2, 2), 4);
    EXPECT_EQ(filtered_img.at<uchar>(2, 3), 2);
    EXPECT_EQ(filtered_img.at<uchar>(2, 4), 2);

    EXPECT_EQ(filtered_img.at<uchar>(3, 1), 5);
    EXPECT_EQ(filtered_img.at<uchar>(3, 2), 3);
    EXPECT_EQ(filtered_img.at<uchar>(3, 3), 2);
    EXPECT_EQ(filtered_img.at<uchar>(3, 4), 2);

    EXPECT_EQ(filtered_img.at<uchar>(4, 1), 5);
    EXPECT_EQ(filtered_img.at<uchar>(4, 2), 5);
    EXPECT_EQ(filtered_img.at<uchar>(4, 3), 6);
    EXPECT_EQ(filtered_img.at<uchar>(4, 4), 5);
}

TEST_F(MedianFilterTest, NaiveMedianFilterWithMask) {
    cv::Mat mask = cv::Mat::ones(6, 6, CV_8U);

    mask.at<uchar>(0, 1) = 0;
    mask.at<uchar>(0, 4) = 0;
    mask.at<uchar>(1, 1) = 0;
    mask.at<uchar>(1, 2) = 0;
    mask.at<uchar>(1, 4) = 0;
    mask.at<uchar>(2, 3) = 0;
    mask.at<uchar>(3, 2) = 0;
    mask.at<uchar>(3, 3) = 0;
    mask.at<uchar>(4, 1) = 0;
    mask.at<uchar>(4, 3) = 0;
    mask.at<uchar>(4, 4) = 0;
    mask.at<uchar>(4, 5) = 0;
    mask.at<uchar>(5, 2) = 0;

    //  print_array(mask, 6, 6);

    cv::Mat filtered_img;
    naiveMedianFilter(img_, filtered_img, mask, 3);

    //  print_array(filtered_img, 6, 6);

    EXPECT_EQ(filtered_img.at<uchar>(1, 1), 2);
    EXPECT_EQ(filtered_img.at<uchar>(1, 2), 2);
    EXPECT_EQ(filtered_img.at<uchar>(1, 3), 2);
    EXPECT_EQ(filtered_img.at<uchar>(1, 4), 3);

    EXPECT_EQ(filtered_img.at<uchar>(2, 1), 1);
    EXPECT_EQ(filtered_img.at<uchar>(2, 2), 2);
    EXPECT_EQ(filtered_img.at<uchar>(2, 3), 2);
    EXPECT_EQ(filtered_img.at<uchar>(2, 4), 5);

    EXPECT_EQ(filtered_img.at<uchar>(3, 1), 5);
    EXPECT_EQ(filtered_img.at<uchar>(3, 2), 3);
    EXPECT_EQ(filtered_img.at<uchar>(3, 3), 3);
    EXPECT_EQ(filtered_img.at<uchar>(3, 4), 5);

    EXPECT_EQ(filtered_img.at<uchar>(4, 1), 5);
    EXPECT_EQ(filtered_img.at<uchar>(4, 2), 5);
    EXPECT_EQ(filtered_img.at<uchar>(4, 3), 6);
    EXPECT_EQ(filtered_img.at<uchar>(4, 4), 7);
}

TEST_F(MedianFilterTest, HuangFilterWithoutMask) {
    // Create random image filled with values for 0 to 255
    int width = 200;
    int height = 200;
    cv::Mat img(height, width, CV_8U);
    rng_.fill(img, cv::RNG::UNIFORM, 0, 256, true);

    cv::Mat filtered_img_naive, filtered_img_huang;
    naiveMedianFilter(img, filtered_img_naive,
                      cv::Mat::ones(height, width, CV_8U), 17);
    huangMedianFilter(img, filtered_img_huang,
                      cv::Mat::ones(height, width, CV_8U), 17);

    const int p = 8;  // 8*2+1 = 17
    for (int y = p; y < height - p; ++y) {
        for (int x = p; x < width - p; ++x) {
            EXPECT_EQ(filtered_img_naive.at<uchar>(y, x),
                      filtered_img_huang.at<uchar>(y, x));
        }
    }
}

TEST_F(MedianFilterTest, HuangFilterWithMask) {
    // Create random image filled with values for 0 to 255
    int width = 200;
    int height = 200;
    cv::Mat img(height, width, CV_8U);
    rng_.fill(img, cv::RNG::UNIFORM, 0, 256, true);

    cv::Mat mask(height, width, CV_8U);
    rng_.fill(mask, cv::RNG::UNIFORM, 0, 2, false);

    //  print_array(img, height, width);
    //  print_array(mask, height, width);

    cv::Mat filtered_img_naive, filtered_img_huang;
    naiveMedianFilter(img, filtered_img_naive, mask, 17);
    huangMedianFilter(img, filtered_img_huang, mask, 17);

    const int p = 8;  // 8*2+1 = 17
    for (int y = p; y < height - p; ++y) {
        for (int x = p; x < width - p; ++x) {
            EXPECT_EQ(filtered_img_naive.at<uchar>(y, x),
                      filtered_img_huang.at<uchar>(y, x));
        }
    }
}

VIKIT_UNITTEST_ENTRYPOINT
