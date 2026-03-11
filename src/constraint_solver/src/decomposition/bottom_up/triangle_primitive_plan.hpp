#ifndef TRIANGLE_PRIMITIVE_PLAN_HPP
#define TRIANGLE_PRIMITIVE_PLAN_HPP

// Custom headers
#include <gcs/decomposition/bottom_up/plan_node.hpp>
#include <structures/general_tree.hpp>
#include <structures/graph_algorithms.hpp>

namespace Gcs {

/**
 * @brief Build a triangle-primitive leaf plan for one merged triangle cluster.
 */
[[nodiscard]] MathUtils::GeneralTree<PlanNode> makeTrianglePrimitivePlan(
    ClusterId cluster,
    const MathUtils::Triangle<ConstraintGraph::NodeIdType>& triangle);

} // namespace Gcs

#endif // TRIANGLE_PRIMITIVE_PLAN_HPP
