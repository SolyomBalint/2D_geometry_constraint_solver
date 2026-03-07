#ifndef PRODUCER_UPDATE_HPP
#define PRODUCER_UPDATE_HPP

// General STD/STL headers
#include <array>
#include <expected>
#include <span>
#include <vector>

// Custom headers
#include "cluster_graph.hpp"
#include "producer_map.hpp"

namespace Gcs {

enum class ProducerUpdateError {
    OutputClusterNotFound,
    MissingInputProducer,
    InvalidTrianglePrimitive,
    OutputProducerAlreadyExists,
};

/**
 * @brief Update producer mapping after one 3-way cluster merge.
 *
 * Handles both cases:
 * - first triangle creation (no input producers yet) -> triangle leaf
 * - composition of existing produced clusters -> Merge3 internal node
 */
[[nodiscard]] std::expected<void, ProducerUpdateError>
updateProducerAfterMergeThree(ClusterId output, std::array<ClusterId, 3> inputs,
    std::array<std::vector<ConstraintGraph::NodeIdType>, 3> inputElements,
    std::span<const ConstraintGraph::NodeIdType> outputElements,
    ProducerMap& producer);

} // namespace Gcs

#endif // PRODUCER_UPDATE_HPP
