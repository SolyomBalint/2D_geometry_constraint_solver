#include "solving/bottom_up/merge3_fallback_solver.hpp"

// General STD/STL headers
#include <array>
#include <unordered_set>

// Custom headers
#include "solving/bottom_up/merge3_solver_common.hpp"

namespace Gcs::Solvers::BottomUp {

bool detectUnsolvableMerge3Lll(const Merge3Context& context)
{
    if (context.node.kind != PlanNodeKind::Merge3) {
        return false;
    }

    for (std::size_t referenceIndex = 0; referenceIndex < 3; ++referenceIndex) {
        std::array<std::size_t, 2> movingIndices {};
        std::size_t movingInsertIndex = 0;
        for (std::size_t index = 0; index < 3; ++index) {
            if (index == referenceIndex) {
                continue;
            }
            movingIndices[movingInsertIndex++] = index;
        }

        const ClusterPose& referenceCluster
            = context.solvedNodePose.at(context.children[referenceIndex]);
        const ClusterPose& movingClusterA
            = context.solvedNodePose.at(context.children[movingIndices[0]]);
        const ClusterPose& movingClusterB
            = context.solvedNodePose.at(context.children[movingIndices[1]]);

        std::unordered_set<ConstraintGraph::NodeIdType> referenceElements;
        for (const auto& [elementId, /*pose*/ _] : referenceCluster) {
            referenceElements.insert(elementId);
        }

        const auto sharedRefALines = clusterIntersectionByType(
            context.sourceGraph, referenceCluster, movingClusterA, false);
        const auto sharedRefBLines = clusterIntersectionByType(
            context.sourceGraph, referenceCluster, movingClusterB, false);
        const auto sharedABLines = clusterIntersectionByType(
            context.sourceGraph, movingClusterA, movingClusterB, false);

        if (sharedRefALines.empty() || sharedRefBLines.empty()) {
            continue;
        }

        for (const auto& freeLineId : sharedABLines) {
            if (!referenceElements.contains(freeLineId)) {
                return true;
            }
        }
    }

    return false;
}

std::optional<ClusterPose> Merge3FallbackSolver::solve(
    const Merge3Context& context)
{
    if (context.node.kind != PlanNodeKind::Merge3) {
        return std::nullopt;
    }

    auto mergedCluster = context.solvedNodePose.at(context.children[0]);
    const auto firstMerge
        = mergeChildClusterIntoReference(std::move(mergedCluster),
            context.solvedNodePose.at(context.children[1]));
    if (!firstMerge.has_value()) {
        return std::nullopt;
    }

    return mergeChildClusterIntoReference(
        firstMerge.value(), context.solvedNodePose.at(context.children[2]));
}

} // namespace Gcs::Solvers::BottomUp
