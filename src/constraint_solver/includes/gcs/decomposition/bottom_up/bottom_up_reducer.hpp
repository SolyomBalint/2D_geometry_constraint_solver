#ifndef GCS_DECOMPOSITION_BOTTOM_UP_BOTTOM_UP_REDUCER_HPP
#define GCS_DECOMPOSITION_BOTTOM_UP_BOTTOM_UP_REDUCER_HPP

// General STD/STL headers
#include <vector>

// Custom headers
#include <gcs/decomposition/bottom_up/producer_map.hpp>
#include <gcs/export.hpp>
#include <gcs/model/gcs_data_structures.hpp>

namespace Gcs {

struct GCS_API BottomUpReductionResult {
    std::vector<ClusterId> remainingClusters;
    std::vector<PlanTree> rootPlans;
};

/**
 * @brief First end-to-end bottom-up reduction pass.
 *
 * This version performs initial edge-cluster creation and one sweep of
 * triangle-induced 6-cycle reductions.
 */
[[nodiscard]] GCS_API BottomUpReductionResult reduceBottomUp(
    const ConstraintGraph& graph);

} // namespace Gcs

#endif // GCS_DECOMPOSITION_BOTTOM_UP_BOTTOM_UP_REDUCER_HPP
