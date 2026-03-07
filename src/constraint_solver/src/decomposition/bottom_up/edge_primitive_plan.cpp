#include "edge_primitive_plan.hpp"

namespace Gcs {

MathUtils::GeneralTree<PlanNode> makeEdgePrimitivePlan(ClusterId cluster,
    std::array<ConstraintGraph::NodeIdType, 2> canonicalEdgeElements)
{
    const PlanNode node {
        .kind = PlanNodeKind::EdgePrimitive,
        .info = EdgePrimitiveInfo {
            .cluster = cluster,
            .elements = canonicalEdgeElements,
        },
    };

    return MathUtils::GeneralTree<PlanNode> { node };
}

} // namespace Gcs
