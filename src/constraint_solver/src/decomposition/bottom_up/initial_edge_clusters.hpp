#ifndef INITIAL_EDGE_CLUSTERS_HPP
#define INITIAL_EDGE_CLUSTERS_HPP

// General STD/STL headers
#include <expected>

// Custom headers
#include "cluster_graph.hpp"
#include <gcs/model/gcs_data_structures.hpp>

namespace Gcs {

/**
 * @brief Create initial size-2 clusters for every constraint-graph edge.
 *
 * For each edge @p (u, v) in the input graph, adds one cluster @p {u, v} to the
 * bipartite cluster graph.
 */
[[nodiscard]] std::expected<void, ClusterGraphError> addInitialEdgeClusters(
    const ConstraintGraph& constraintGraph, ClusterGraph& clusterGraph);

} // namespace Gcs

#endif // INITIAL_EDGE_CLUSTERS_HPP
