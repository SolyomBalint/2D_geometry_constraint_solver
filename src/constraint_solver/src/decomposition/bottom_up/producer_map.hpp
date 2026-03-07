#ifndef PRODUCER_MAP_HPP
#define PRODUCER_MAP_HPP

// General STD/STL headers
#include <unordered_map>

// Custom headers
#include "plan_node.hpp"
#include <structures/general_tree.hpp>

namespace Gcs {

using PlanTree = MathUtils::GeneralTree<PlanNode>;
using ProducerMap = std::unordered_map<ClusterId, PlanTree>;

} // namespace Gcs

#endif // PRODUCER_MAP_HPP
