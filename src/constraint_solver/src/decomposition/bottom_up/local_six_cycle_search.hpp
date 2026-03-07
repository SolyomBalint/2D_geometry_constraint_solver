#ifndef LOCAL_SIX_CYCLE_SEARCH_HPP
#define LOCAL_SIX_CYCLE_SEARCH_HPP

// General STD/STL headers
#include <vector>

// Custom headers
#include "cluster_graph.hpp"
#include "six_cycle_witness.hpp"

namespace Gcs {

/**
 * @brief Find local 6-cycles around one cluster using depth-limited BFS.
 *
 * The search starts at @p clusterId and explores alternating levels in H only
 * up to depth 3. A level-3 element reached through two distinct level-2
 * cluster branches yields a 6-cycle witness.
 */
[[nodiscard]] std::vector<SixCycleWitness> findLocalSixCyclesAround(
    ClusterId clusterId, const ClusterGraph& clusterGraph);

} // namespace Gcs

#endif // LOCAL_SIX_CYCLE_SEARCH_HPP
