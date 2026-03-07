#include "triangle_primitive_plan.hpp"

// General STD/STL headers
#include <algorithm>
#include <array>

namespace Gcs {

namespace {

    // Convert a triangle to canonical ascending element array.
    [[nodiscard]] std::array<ConstraintGraph::NodeIdType, 3>
    canonicalTriangleArray(
        const MathUtils::Triangle<ConstraintGraph::NodeIdType>& triangle)
    {
        std::array<ConstraintGraph::NodeIdType, 3> elements {
            triangle.a,
            triangle.b,
            triangle.c,
        };
        std::ranges::sort(elements);
        return elements;
    }

} // namespace

MathUtils::GeneralTree<PlanNode> makeTrianglePrimitivePlan(ClusterId cluster,
    const MathUtils::Triangle<ConstraintGraph::NodeIdType>& triangle)
{
    const PlanNode node {
        .kind = PlanNodeKind::TrianglePrimitive,
        .info = TrianglePrimitiveInfo {
            .cluster = cluster,
            .elements = canonicalTriangleArray(triangle),
        },
    };

    return MathUtils::GeneralTree<PlanNode> { node };
}

} // namespace Gcs
