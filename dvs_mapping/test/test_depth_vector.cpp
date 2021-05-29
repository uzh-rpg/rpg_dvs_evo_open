#include <aslam/common/entrypoint.h>
#include <gtest/gtest.h>

#include <dvs_depth_from_defocus/depth_vector.hpp>

using namespace depth_from_defocus;

TEST(DepthVectorTest, LinearDepth) {
    size_t num_depth_cells = 5;
    evo_utils::geometry::Depth min_depth = 1.0;
    evo_utils::geometry::Depth max_depth = 5.0;
    DepthVector depth_vec(min_depth, max_depth, num_depth_cells);

    EXPECT_NEAR(depth_vec.cellIndexToDepth(0), 1.0, 1e-4);
    EXPECT_NEAR(depth_vec.cellIndexToDepth(1), 2.0, 1e-4);
    EXPECT_NEAR(depth_vec.cellIndexToDepth(2), 3.0, 1e-4);
    EXPECT_NEAR(depth_vec.cellIndexToDepth(3), 4.0, 1e-4);
    EXPECT_NEAR(depth_vec.cellIndexToDepth(4), 5.0, 1e-4);
}

TEST(DepthVectorTest, ForwardBackward) {
    size_t num_tests = 5;
    size_t num_depth_cells = 10;

    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_real_distribution<double> distr(1.0, 50.0);

    // depth
    for (size_t n = 0; n < num_tests; ++n) {
        evo_utils::geometry::Depth min_depth = distr(generator);
        evo_utils::geometry::Depth max_depth = distr(generator);
        DepthVector depth_vector(min_depth, max_depth, num_depth_cells);

        for (size_t i = 0; i < depth_vector.size(); ++i) {
            const evo_utils::geometry::Depth d_i =
                depth_vector.cellIndexToDepth(i);
            const size_t rec_i = depth_vector.depthToCellIndex(d_i);
            EXPECT_EQ(i, rec_i) << "Forward-Backward index mismatch";
        }
    }
}
