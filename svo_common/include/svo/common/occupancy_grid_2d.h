#pragma once

#include <svo/common/types.h>
#include <glog/logging.h>

namespace svo {
	
/// We divide the image into a grid of cells and try to find maximally one
/// feature per cell. This is to ensure good distribution of features in the
/// image.
class OccupandyGrid2D
{
public:
  using Grid = std::vector<bool>;

  OccupandyGrid2D(int cell_size, int n_cols, int n_rows)
    : cell_size(cell_size)
    , n_cols(n_cols)
    , n_rows(n_rows)
    , occupancy_(n_cols*n_rows, false)
  {}
  ~OccupandyGrid2D() = default;

  const int cell_size;
  const int n_cols;
  const int n_rows;
  Grid occupancy_;

  inline void reset() {
    std::fill(occupancy_.begin(), occupancy_.end(), false);
  }

  inline size_t size() {
    return occupancy_.size();
  }

  inline bool empty() {
    return occupancy_.empty();
  }

  inline bool isOccupied(const size_t cell_index)
  {
    CHECK_LT(cell_index, occupancy_.size());
    return occupancy_[cell_index];
  }

  inline void setOccupied(const size_t cell_index)
  {
    CHECK_LT(cell_index, occupancy_.size());
    occupancy_[cell_index] = true;
  }

  template<typename Derived>
  size_t getCellIndex(const Eigen::MatrixBase<Derived>& px)
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 2, 1);
    return static_cast<size_t>((px(1))/cell_size*n_cols
                             + (px(0))/cell_size);
  }

  inline size_t getCellIndex(int x, int y, int scale = 1) const
  {
    return static_cast<size_t>((scale*y)/cell_size*n_cols
                             + (scale*x)/cell_size);
  }

  inline void fillWithKeypoints(const Keypoints& keypoints)
  {
    // TODO(cfo): could be implemented using block operations.
    for(int i = 0; i < keypoints.cols(); ++i)
      occupancy_.at(getCellIndex(static_cast<int>(keypoints(0,i)),
                                 static_cast<int>(keypoints(1,i)), 1)) = true;
  }
};

} // namespace svo
