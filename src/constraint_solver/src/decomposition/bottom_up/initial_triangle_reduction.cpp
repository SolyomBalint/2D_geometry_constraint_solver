#include "initial_triangle_reduction.hpp"

// General STD/STL headers
#include <cassert>

// Custom headers
#include "six_cycle_witness.hpp"
#include "triangle_primitive_plan.hpp"

namespace Gcs {

std::expected<std::optional<ClusterId>, ClusterGraphError>
reduceInitialTriangleToPrimitive(
    const MathUtils::Triangle<ConstraintGraph::NodeIdType>& triangle,
    ClusterGraph& clusterGraph, ProducerMap& producer)
{
    const auto witness = findInitialSixCycleForTriangle(triangle, clusterGraph);
    if (!witness.has_value()) {
        return std::optional<ClusterId> {};
    }

    const auto mergedCluster = clusterGraph.mergeThreeClusters(
        witness->ab, witness->bc, witness->ac);
    if (!mergedCluster.has_value()) {
        return std::unexpected(mergedCluster.error());
    }

    const auto [it, inserted] = producer.emplace(mergedCluster.value(),
        makeTrianglePrimitivePlan(mergedCluster.value(), triangle));
    (void)it;
    assert(inserted && "Merged cluster must get a new producer leaf");

    return std::optional<ClusterId> { mergedCluster.value() };
}

} // namespace Gcs
