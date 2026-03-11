#ifndef GCS_DECOMPOSITION_BOTTOM_UP_PRODUCER_MAP_HPP
#define GCS_DECOMPOSITION_BOTTOM_UP_PRODUCER_MAP_HPP

// General STD/STL headers
#include <unordered_map>

// Custom headers
#include <gcs/decomposition/bottom_up/plan_node.hpp>
#include <structures/general_tree.hpp>

namespace Gcs {

using PlanTree = MathUtils::GeneralTree<PlanNode>;
using ProducerMap = std::unordered_map<ClusterId, PlanTree>;

} // namespace Gcs

#endif // GCS_DECOMPOSITION_BOTTOM_UP_PRODUCER_MAP_HPP
