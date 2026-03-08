#ifndef MERGE3_CASE_SOLVER_HPP
#define MERGE3_CASE_SOLVER_HPP

// General STD/STL headers
#include <concepts>
#include <optional>
#include <span>
#include <unordered_map>

// Custom headers
#include "decomposition/bottom_up/plan_node.hpp"
#include "solving/bottom_up/plan_pose_types.hpp"
#include <structures/general_tree.hpp>

namespace Gcs::Solvers::BottomUp {

struct Merge3Context {
    const ConstraintGraph& sourceGraph;
    const PlanNode& node;
    std::span<const MathUtils::GeneralTreeNodeId> children;
    const std::unordered_map<MathUtils::GeneralTreeNodeId, ClusterPose>&
        solvedNodePose;
};

template <typename T>
concept Merge3CaseSolver = requires(const Merge3Context& context) {
    { T::solve(context) } -> std::same_as<std::optional<ClusterPose>>;
};

} // namespace Gcs::Solvers::BottomUp

#endif // MERGE3_CASE_SOLVER_HPP
