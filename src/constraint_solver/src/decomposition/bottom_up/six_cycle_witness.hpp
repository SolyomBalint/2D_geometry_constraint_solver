#ifndef SIX_CYCLE_WITNESS_HPP
#define SIX_CYCLE_WITNESS_HPP

// General STD/STL headers
#include <optional>

// Custom headers
#include "cluster_graph.hpp"
#include <structures/graph_algorithms.hpp>

namespace Gcs {

struct SixCycleWitness {
    ClusterId ab;
    ClusterId bc;
    ClusterId ac;

    ConstraintGraph::NodeIdType a;
    ConstraintGraph::NodeIdType b;
    ConstraintGraph::NodeIdType c;
};

/**
 * @brief Locate the initial 6-cycle witness for one triangle.
 *
 * For triangle (a,b,c), this looks up the three size-2 clusters {a,b}, {b,c},
 * and {a,c} in the current cluster graph.
 *
 * @return A witness if all three pair-clusters exist unambiguously,
 *         @c std::nullopt otherwise.
 */
[[nodiscard]] std::optional<SixCycleWitness> findInitialSixCycleForTriangle(
    const MathUtils::Triangle<ConstraintGraph::NodeIdType>& triangle,
    const ClusterGraph& clusterGraph);

} // namespace Gcs

#endif // SIX_CYCLE_WITNESS_HPP
