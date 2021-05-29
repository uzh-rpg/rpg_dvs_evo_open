#include "evo_utils/geometry.hpp"

namespace depth_from_defocus {

// Stores a set of data associations index <-> depth
// Used to abstract how the depth is sampled

// Uses the "Curiously Recursive Template Pattern" to allow static polymorphism
// for performance (since these functions will be called very often)

// TODO: scope the actual benefit of using this pattern compared to inheritance
// (no polymorphism needed..)

template <class DerivedDepthVector>
/**
 * Stores a set of data associations index <-> depth
 * Used to abstract how the depth is sampled
 */
class DepthVector {
   public:
    DerivedDepthVector& derived() {
        return static_cast<DerivedDepthVector&>(*this);
    }

    DepthVector() {}

    /**
     * Create depth vector
     *
     * @param min_depth Lower bound on depth range
     * @param max_depth Upper bound on depth range
     * @param num_depth_cells Number of descrete cells in the depth axis
     *
     * @pre min_depth > 0
     * @pre max_depth > 0
     * @pre max_depth > min_depth
     * @pre num_depth_cells >= 1
     */
    DepthVector(float min_depth, float max_depth, size_t num_depth_cells) {
        CHECK_GT(min_depth, 0.0);
        CHECK_GT(max_depth, 0.0);
        CHECK_GE(num_depth_cells, 1u);

        num_depth_cells_ = num_depth_cells;
        min_depth_ = min_depth;
        max_depth_ = max_depth;
        if (min_depth_ > max_depth_) std::swap(min_depth_, max_depth_);

        derived().init();
    }

    /**
     * Number of discrete cells in the depth axis
     *
     * @return number of descrete cells in the depth axis
     */
    inline size_t size() const { return vec_.size(); }

    /**
     * DSI util function: from cell index to depth
     *
     * The depth axis is evenly divided in DepthVector::Size cells, and
     * ranges within DepthVector::min_depth_ and DepthVector::max_depth_. The
     * conversion is a linear interpolation.
     *
     * @param i Cell index in the discrete depth axis
     */
    float cellIndexToDepth(size_t i) { return derived().cellIndexToDepth(i); }

    /**
     * DSI util function: from depth to cell index
     *
     * The depth axis is evenly divided in DepthVector::Size cells, and
     * ranges within DepthVector::min_depth_ and DepthVector::max_depth_. The
     * conversion is rounding to the closest cell index.
     *
     * @param depth Depth to convert
     */
    size_t depthToCellIndex(float depth) {
        return derived().depthToCellIndex(depth);
    }

    /**
     * DSI util function: from depth to cell value
     *
     * @param depth depth depth to convert
     *
     * @see depthToCellIndex
     */
    float depthToCell(float depth) { return derived().depthToCell(depth); }

    /**
     * Get the DepthVector container
     *
     * @return vector s.t. v[i] is the depth value for the i-th cell
     */
    std::vector<float> getDepthVector() {
        std::vector<float> out;
        for (size_t i = 0; i < num_depth_cells_; ++i) {
            out.push_back(cellIndexToDepth(i));
        }
        return out;
    }

   protected:
    std::vector<float> vec_;
    size_t num_depth_cells_;
    float min_depth_;
    float max_depth_;
    float depth_to_cell_idx_multiplier_;
};

/**
 * DepthVector dealing with linear depth axis
 */
class LinearDepthVector : public DepthVector<LinearDepthVector> {
   public:
    LinearDepthVector() : DepthVector() {}

    LinearDepthVector(float min_depth, float max_depth, size_t num_depth_cells)
        : DepthVector(min_depth, max_depth, num_depth_cells) {}

    void init() {
        depth_to_cell_idx_multiplier_ =
            (float)((num_depth_cells_ - 1) / (max_depth_ - min_depth_));

        vec_.resize(num_depth_cells_);
        for (size_t i = 0; i < num_depth_cells_; ++i) {
            vec_[i] = min_depth_ + (float)i / depth_to_cell_idx_multiplier_;
        }
    }

    float cellIndexToDepth(size_t i) { return vec_[i]; }

    size_t depthToCellIndex(float depth) {
        return (size_t)((depth - min_depth_) * depth_to_cell_idx_multiplier_ +
                        0.5);
    }

    float depthToCell(float depth) {
        return (depth - min_depth_) * depth_to_cell_idx_multiplier_;
    }
};

/**
 * DepthVector dealing with inverse depth (1/d) axis
 */
class InverseDepthVector : public DepthVector<InverseDepthVector> {
   public:
    InverseDepthVector() : DepthVector() {}

    InverseDepthVector(float min_depth, float max_depth, size_t num_depth_cells)
        : DepthVector(min_depth, max_depth, num_depth_cells) {}

    void init() {
        inv_min_depth_ = 1.f / min_depth_;
        inv_max_depth_ = 1.f / max_depth_;
        depth_to_cell_idx_multiplier_ =
            (float)((num_depth_cells_ - 1) / (inv_min_depth_ - inv_max_depth_));

        vec_.resize(num_depth_cells_);
        for (size_t i = 0; i < num_depth_cells_; ++i) {
            vec_[i] = inv_max_depth_ + (float)i / depth_to_cell_idx_multiplier_;
        }
    }

    float cellIndexToDepth(size_t i) { return 1.f / vec_[i]; }

    size_t depthToCellIndex(float depth) {
        return (size_t)((1.f / depth - inv_max_depth_) *
                            depth_to_cell_idx_multiplier_ +
                        0.5);
    }

    float depthToCell(float depth) {
        return (1.f / depth - inv_max_depth_) * depth_to_cell_idx_multiplier_;
    }

   private:
    float inv_min_depth_;
    float inv_max_depth_;
};

}  // namespace depth_from_defocus
