#include "solving/bottom_up/merge3_pll_solver.hpp"

// General STD/STL headers
#include <array>
#include <format>
#include <iostream>
#include <limits>
#include <unordered_set>

// Custom headers
#include "solving/bottom_up/merge3_solver_common.hpp"

namespace Gcs::Solvers::BottomUp {

std::optional<ClusterPose> Merge3PllSolver::solve(const Merge3Context& context)
{
    if (context.node.kind != PlanNodeKind::Merge3) {
        return std::nullopt;
    }

    const auto mergeInfo = std::get<Merge3Info>(context.node.info);
    std::optional<ClusterPose> bestMergedPose;
    double bestScore = std::numeric_limits<double>::infinity();

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

        const auto sharedRefAPoints = clusterIntersectionByType(
            context.sourceGraph, referenceCluster, movingClusterA, true);
        const auto sharedRefBPoints = clusterIntersectionByType(
            context.sourceGraph, referenceCluster, movingClusterB, true);
        const auto sharedABLines = clusterIntersectionByType(
            context.sourceGraph, movingClusterA, movingClusterB, false);

        std::vector<ConstraintGraph::NodeIdType> freeLineCandidates;
        for (const auto& lineId : sharedABLines) {
            if (!referenceElements.contains(lineId)) {
                freeLineCandidates.push_back(lineId);
            }
        }

        for (const auto& fixedPointA : sharedRefAPoints) {
            for (const auto& fixedPointB : sharedRefBPoints) {
                if (fixedPointA == fixedPointB) {
                    continue;
                }

                const auto fixedAInGlobal
                    = getPointPosition(referenceCluster, fixedPointA);
                const auto fixedBInGlobal
                    = getPointPosition(referenceCluster, fixedPointB);
                const auto fixedACanvas
                    = getPointCanvasPosition(context.sourceGraph, fixedPointA);
                const auto fixedBCanvas
                    = getPointCanvasPosition(context.sourceGraph, fixedPointB);
                if (!fixedAInGlobal.has_value() || !fixedBInGlobal.has_value()
                    || !fixedACanvas.has_value() || !fixedBCanvas.has_value()) {
                    continue;
                }

                for (const auto& freeLineId : freeLineCandidates) {
                    const auto freeLineCanvas
                        = getLineCanvasPose(context.sourceGraph, freeLineId);
                    const auto freeLineInMovingA
                        = getLinePosition(movingClusterA, freeLineId);
                    const auto freeLineInMovingB
                        = getLinePosition(movingClusterB, freeLineId);
                    const auto fixedAInMovingA
                        = getPointPosition(movingClusterA, fixedPointA);
                    const auto fixedBInMovingB
                        = getPointPosition(movingClusterB, fixedPointB);
                    if (!freeLineCanvas.has_value()
                        || !freeLineInMovingA.has_value()
                        || !freeLineInMovingB.has_value()
                        || !fixedAInMovingA.has_value()
                        || !fixedBInMovingB.has_value()) {
                        continue;
                    }

                    const double distanceA = pointToLineDistanceAbs(
                        fixedAInMovingA.value(), freeLineInMovingA.value());
                    const double distanceB = pointToLineDistanceAbs(
                        fixedBInMovingB.value(), freeLineInMovingB.value());

                    const auto solvedFreeLine = solveFreeLineFromFixedPoints(
                        fixedAInGlobal.value(), fixedBInGlobal.value(),
                        distanceA, distanceB, fixedACanvas.value(),
                        fixedBCanvas.value(), freeLineCanvas.value());
                    if (!solvedFreeLine.has_value()) {
                        continue;
                    }

                    const std::array<
                        std::pair<ConstraintGraph::NodeIdType, ElementPose>, 2>
                        anchorsA {
                            std::pair<ConstraintGraph::NodeIdType,
                                ElementPose> { fixedPointA,
                                PointPose {
                                    .position = fixedAInGlobal.value(),
                                } },
                            std::pair<ConstraintGraph::NodeIdType,
                                ElementPose> {
                                freeLineId,
                                solvedFreeLine.value(),
                            },
                        };
                    const std::array<
                        std::pair<ConstraintGraph::NodeIdType, ElementPose>, 2>
                        anchorsB {
                            std::pair<ConstraintGraph::NodeIdType,
                                ElementPose> { fixedPointB,
                                PointPose {
                                    .position = fixedBInGlobal.value(),
                                } },
                            std::pair<ConstraintGraph::NodeIdType,
                                ElementPose> {
                                freeLineId,
                                solvedFreeLine.value(),
                            },
                        };

                    const auto transformedA
                        = transformClusterByAnchors(movingClusterA, anchorsA);
                    const auto transformedB
                        = transformClusterByAnchors(movingClusterB, anchorsB);
                    if (!transformedA.has_value()
                        || !transformedB.has_value()) {
                        continue;
                    }

                    ClusterPose merged = referenceCluster;
                    merged[freeLineId] = solvedFreeLine.value();

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

                    const double score
                        = scoreMergedPose(context.sourceGraph, merged);
                    std::cerr << std::format(
                        "[BottomUp][Merge3][C{}] PLL candidate ref={} "
                        "fixedA={} fixedB={} freeLine={} score={}\n",
                        mergeInfo.output.value, referenceIndex,
                        fixedPointA.value, fixedPointB.value, freeLineId.value,
                        score);

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
            "[BottomUp][Merge3][C{}] Selected PLL candidate score={}\n",
            mergeInfo.output.value, bestScore);
    }

    return bestMergedPose;
}

} // namespace Gcs::Solvers::BottomUp
