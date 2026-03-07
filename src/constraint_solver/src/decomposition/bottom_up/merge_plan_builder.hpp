#ifndef MERGE_PLAN_BUILDER_HPP
#define MERGE_PLAN_BUILDER_HPP

// General STD/STL headers
#include <array>
#include <span>

// Custom headers
#include "plan_node.hpp"
#include <structures/general_tree.hpp>

namespace Gcs {

/**
 * @brief Build a Merge3 plan node with three child subplans.
 *
 * Input cluster ordering is canonicalized to keep output deterministic.
 */
[[nodiscard]] MathUtils::GeneralTree<PlanNode> makeMerge3Plan(ClusterId output,
    std::array<ClusterId, 3> inputs,
    std::array<MathUtils::GeneralTree<PlanNode>, 3> childPlans,
    std::span<const ConstraintGraph::NodeIdType> outputElements = {});

} // namespace Gcs

#endif // MERGE_PLAN_BUILDER_HPP
