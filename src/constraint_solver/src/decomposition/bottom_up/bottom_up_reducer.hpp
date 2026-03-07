#ifndef BOTTOM_UP_REDUCER_HPP
#define BOTTOM_UP_REDUCER_HPP

// General STD/STL headers
#include <vector>

// Custom headers
#include "decomposition/bottom_up/producer_map.hpp"
#include "model/gcs_data_structures.hpp"

namespace Gcs {

struct BottomUpReductionResult {
    std::vector<ClusterId> remainingClusters;
    std::vector<PlanTree> rootPlans;
};

/**
 * @brief First end-to-end bottom-up reduction pass.
 *
 * This version performs initial edge-cluster creation and one sweep of
 * triangle-induced 6-cycle reductions.
 */
[[nodiscard]] BottomUpReductionResult reduceBottomUp(
    const ConstraintGraph& graph);

} // namespace Gcs

#endif // BOTTOM_UP_REDUCER_HPP
