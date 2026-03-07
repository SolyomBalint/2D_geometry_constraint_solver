#include "initial_edge_clusters.hpp"

// General STD/STL headers
#include <array>
#include <cassert>
#include <expected>

namespace Gcs {

std::expected<void, ClusterGraphError> addInitialEdgeClusters(
    const ConstraintGraph& constraintGraph, ClusterGraph& clusterGraph)
{
    for (const auto& edgeId : constraintGraph.getGraph().getEdges()) {
        const auto [u, v] = constraintGraph.getGraph().getEndpoints(edgeId);
        assert(u != v && "Initial edge clusters require non-loop edges");

        const std::array<ConstraintGraph::NodeIdType, 2> edgeElements { u, v };
        const auto clusterId = clusterGraph.addCluster(edgeElements);
        if (!clusterId.has_value()) {
            return std::unexpected(clusterId.error());
        }
    }

    return {};
}

} // namespace Gcs
