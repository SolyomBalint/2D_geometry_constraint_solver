#ifndef INITIAL_TRIANGLE_REDUCTION_HPP
#define INITIAL_TRIANGLE_REDUCTION_HPP

// General STD/STL headers
#include <expected>
#include <optional>

// Custom headers
#include "cluster_graph.hpp"
#include "producer_map.hpp"
#include <structures/graph_algorithms.hpp>

namespace Gcs {

/**
 * @brief Reduce one initial triangle and store its primitive producer leaf.
 *
 * If the corresponding 6-cycle is currently present in @p clusterGraph, merges
 * it and inserts a triangle primitive leaf for the newly created cluster into
 * @p producer.
 *
 * @return New merged cluster ID on success, @c std::nullopt if no valid initial
 *         6-cycle exists for this triangle, or an error if merge fails.
 */
[[nodiscard]] std::expected<std::optional<ClusterId>, ClusterGraphError>
reduceInitialTriangleToPrimitive(
    const MathUtils::Triangle<ConstraintGraph::NodeIdType>& triangle,
    ClusterGraph& clusterGraph, ProducerMap& producer);

} // namespace Gcs

#endif // INITIAL_TRIANGLE_REDUCTION_HPP
