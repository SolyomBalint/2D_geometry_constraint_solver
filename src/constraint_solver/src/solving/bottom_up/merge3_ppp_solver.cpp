#include "solving/bottom_up/merge3_ppp_solver.hpp"

// General STD/STL headers
#include <array>
#include <format>
#include <iostream>
#include <limits>
#include <unordered_set>

// Custom headers
#include "solving/bottom_up/merge3_solver_common.hpp"
#include "solving/equations/equation_primitives.hpp"
#include "solving/equations/newton_raphson.hpp"
#include "solving/solvers/heuristics.hpp"

namespace Gcs::Solvers::BottomUp {

std::optional<ClusterPose> Merge3PppSolver::solve(const Merge3Context& context)
{
    if (context.node.kind != PlanNodeKind::Merge3) {
        return std::nullopt;
    }

    const auto mergeInfo = std::get<Merge3Info>(context.node.info);

    std::optional<ClusterPose> bestMergedPose;
    double bestScore = std::numeric_limits<double>::infinity();
    int attemptedCandidates = 0;

    std::cerr << std::format(
        "[BottomUp][Merge3][C{}] Trying pairwise-anchor PPP merge\n",
        mergeInfo.output.value);

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
        referenceElements.reserve(referenceCluster.size());
        for (const auto& [elementId, /*pose*/ _] : referenceCluster) {
            referenceElements.insert(elementId);
        }

        const auto sharedRefA = clusterIntersectionByType(
            context.sourceGraph, referenceCluster, movingClusterA, true);
        const auto sharedRefB = clusterIntersectionByType(
            context.sourceGraph, referenceCluster, movingClusterB, true);
        const auto sharedAB = clusterIntersectionByType(
            context.sourceGraph, movingClusterA, movingClusterB, true);

        std::vector<ConstraintGraph::NodeIdType> freeCandidates;
        for (const auto& elementId : sharedAB) {
            if (!referenceElements.contains(elementId)) {
                freeCandidates.push_back(elementId);
            }
        }

        std::cerr << std::format(
            "[BottomUp][Merge3][C{}] ref={} | shared(ref,A)={} "
            "shared(ref,B)={} shared(A,B)={} freeCandidates={}\n",
            mergeInfo.output.value, referenceIndex, sharedRefA.size(),
            sharedRefB.size(), sharedAB.size(), freeCandidates.size());

        for (const auto& fixedPointA : sharedRefA) {
            for (const auto& fixedPointB : sharedRefB) {
                if (fixedPointA == fixedPointB) {
                    continue;
                }

                const auto fixedAInGlobal
                    = getPointPosition(referenceCluster, fixedPointA);
                const auto fixedBInGlobal
                    = getPointPosition(referenceCluster, fixedPointB);
                if (!fixedAInGlobal.has_value()
                    || !fixedBInGlobal.has_value()) {
                    continue;
                }

                for (const auto& freePointId : freeCandidates) {
                    if (freePointId == fixedPointA
                        || freePointId == fixedPointB) {
                        continue;
                    }

                    const auto fixedAInMovingA
                        = getPointPosition(movingClusterA, fixedPointA);
                    const auto freeInMovingA
                        = getPointPosition(movingClusterA, freePointId);
                    const auto fixedBInMovingB
                        = getPointPosition(movingClusterB, fixedPointB);
                    const auto freeInMovingB
                        = getPointPosition(movingClusterB, freePointId);

                    if (!fixedAInMovingA.has_value()
                        || !freeInMovingA.has_value()
                        || !fixedBInMovingB.has_value()
                        || !freeInMovingB.has_value()) {
                        continue;
                    }

                    const double distanceAFree
                        = (fixedAInMovingA.value() - freeInMovingA.value())
                              .norm();
                    const double distanceBFree
                        = (fixedBInMovingB.value() - freeInMovingB.value())
                              .norm();
                    if (distanceAFree < EPSILON || distanceBFree < EPSILON) {
                        continue;
                    }

                    const auto fixedACanvas = getPointCanvasPosition(
                        context.sourceGraph, fixedPointA);
                    const auto fixedBCanvas = getPointCanvasPosition(
                        context.sourceGraph, fixedPointB);
                    const auto freeCanvas = getPointCanvasPosition(
                        context.sourceGraph, freePointId);
                    if (!fixedACanvas.has_value() || !fixedBCanvas.has_value()
                        || !freeCanvas.has_value()) {
                        continue;
                    }

                    auto distanceFromAFunction
                        = Equations::pointToPointDistance(fixedAInGlobal->x(),
                            fixedAInGlobal->y(), distanceAFree);
                    auto distanceFromBFunction
                        = Equations::pointToPointDistance(fixedBInGlobal->x(),
                            fixedBInGlobal->y(), distanceBFree);

                    const auto possibleFreePositions = Equations::solve2D(
                        distanceFromAFunction, distanceFromBFunction);

                    const Eigen::Vector2d selectedFreePoint
                        = Solvers::pickByTriangleOrientation(
                            fixedACanvas.value(), fixedBCanvas.value(),
                            freeCanvas.value(), fixedAInGlobal.value(),
                            fixedBInGlobal.value(), possibleFreePositions[0],
                            possibleFreePositions[1]);

                    const auto transformedA = transformClusterByTwoPointAnchors(
                        movingClusterA, fixedPointA, freePointId,
                        fixedAInGlobal.value(), selectedFreePoint);
                    const auto transformedB = transformClusterByTwoPointAnchors(
                        movingClusterB, fixedPointB, freePointId,
                        fixedBInGlobal.value(), selectedFreePoint);
                    if (!transformedA.has_value()
                        || !transformedB.has_value()) {
                        continue;
                    }

                    ClusterPose merged = referenceCluster;
                    merged[freePointId]
                        = PointPose { .position = selectedFreePoint };

                    for (const auto& [elementId, pose] : transformedA.value()) {
                        if (!merged.contains(elementId)) {
                            merged.emplace(elementId, pose);
                        }
                    }
                    for (const auto& [elementId, pose] : transformedB.value()) {
                        if (!merged.contains(elementId)) {
                            merged.emplace(elementId, pose);
                        }
                    }

                    ++attemptedCandidates;
                    const double score
                        = scoreMergedPose(context.sourceGraph, merged);
                    std::cerr << std::format(
                        "[BottomUp][Merge3][C{}] candidate #{} "
                        "ref={} fixedA={} fixedB={} free={} score={}\n",
                        mergeInfo.output.value, attemptedCandidates,
                        referenceIndex, fixedPointA.value, fixedPointB.value,
                        freePointId.value, score);

                    if (score < bestScore) {
                        bestScore = score;
                        bestMergedPose = std::move(merged);
                    }
                }
            }
        }
    }

    if (bestMergedPose.has_value()) {
        std::cerr << std::format(
            "[BottomUp][Merge3][C{}] Selected PPP pairwise candidate "
            "with score={} ({} candidates tried)\n",
            mergeInfo.output.value, bestScore, attemptedCandidates);
    } else {
        std::cerr << std::format(
            "[BottomUp][Merge3][C{}] No valid PPP pairwise candidate "
            "found; falling back to rigid merge\n",
            mergeInfo.output.value);
    }

    return bestMergedPose;
}

} // namespace Gcs::Solvers::BottomUp
