#ifndef EDGE_PRIMITIVE_PLAN_HPP
#define EDGE_PRIMITIVE_PLAN_HPP

// General STD/STL headers
#include <array>

// Custom headers
#include <gcs/decomposition/bottom_up/plan_node.hpp>
#include <structures/general_tree.hpp>

namespace Gcs {

[[nodiscard]] MathUtils::GeneralTree<PlanNode> makeEdgePrimitivePlan(
    ClusterId cluster,
    std::array<ConstraintGraph::NodeIdType, 2> canonicalEdgeElements);

} // namespace Gcs

#endif // EDGE_PRIMITIVE_PLAN_HPP
