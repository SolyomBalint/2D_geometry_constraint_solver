#include "solving/bottom_up/merge3_llp_solver.hpp"

// General STD/STL headers
#include <array>
#include <format>
#include <iostream>
#include <limits>
#include <unordered_set>

// Custom headers
#include "solving/bottom_up/merge3_solver_common.hpp"

namespace Gcs::Solvers::BottomUp {

std::optional<ClusterPose> Merge3LlpSolver::solve(const Merge3Context& context)
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

        const auto sharedRefALines = clusterIntersectionByType(
            context.sourceGraph, referenceCluster, movingClusterA, false);
        const auto sharedRefBLines = clusterIntersectionByType(
            context.sourceGraph, referenceCluster, movingClusterB, false);
        const auto sharedABPoints = clusterIntersectionByType(
            context.sourceGraph, movingClusterA, movingClusterB, true);

        for (const auto& fixedLineAId : sharedRefALines) {
            for (const auto& fixedLineBId : sharedRefBLines) {
                if (fixedLineAId == fixedLineBId) {
                    continue;
                }

                const auto fixedLineAGlobal
                    = getLinePosition(referenceCluster, fixedLineAId);
                const auto fixedLineBGlobal
                    = getLinePosition(referenceCluster, fixedLineBId);
                const auto fixedLineACanvas
                    = getLineCanvasPose(context.sourceGraph, fixedLineAId);
                const auto fixedLineBCanvas
                    = getLineCanvasPose(context.sourceGraph, fixedLineBId);
                if (!fixedLineAGlobal.has_value()
                    || !fixedLineBGlobal.has_value()
                    || !fixedLineACanvas.has_value()
                    || !fixedLineBCanvas.has_value()) {
                    continue;
                }

                for (const auto& freePointId : sharedABPoints) {
                    if (referenceElements.contains(freePointId)) {
                        continue;
                    }

                    const auto freePointInA
                        = getPointPosition(movingClusterA, freePointId);
                    const auto freePointInB
                        = getPointPosition(movingClusterB, freePointId);
                    const auto fixedLineAInA
                        = getLinePosition(movingClusterA, fixedLineAId);
                    const auto fixedLineBInB
                        = getLinePosition(movingClusterB, fixedLineBId);
                    const auto freePointCanvas = getPointCanvasPosition(
                        context.sourceGraph, freePointId);

                    if (!freePointInA.has_value() || !freePointInB.has_value()
                        || !fixedLineAInA.has_value()
                        || !fixedLineBInB.has_value()
                        || !freePointCanvas.has_value()) {
                        continue;
                    }

                    const double distanceToA = pointToLineDistanceAbs(
                        freePointInA.value(), fixedLineAInA.value());
                    const double distanceToB = pointToLineDistanceAbs(
                        freePointInB.value(), fixedLineBInB.value());

                    const auto solvedFreePoint = solveFreePointFromFixedLines(
                        fixedLineAGlobal.value(), fixedLineBGlobal.value(),
                        distanceToA, distanceToB, fixedLineACanvas.value(),
                        fixedLineBCanvas.value(), freePointCanvas.value());
                    if (!solvedFreePoint.has_value()) {
                        continue;
                    }

                    const std::array<
                        std::pair<ConstraintGraph::NodeIdType, ElementPose>, 2>
                        anchorsA {
                            std::pair<ConstraintGraph::NodeIdType,
                                ElementPose> {
                                fixedLineAId,
                                fixedLineAGlobal.value(),
                            },
                            std::pair<ConstraintGraph::NodeIdType,
                                ElementPose> { freePointId,
                                PointPose {
                                    .position = solvedFreePoint.value(),
                                } },
                        };
                    const std::array<
                        std::pair<ConstraintGraph::NodeIdType, ElementPose>, 2>
                        anchorsB {
                            std::pair<ConstraintGraph::NodeIdType,
                                ElementPose> {
                                fixedLineBId,
                                fixedLineBGlobal.value(),
                            },
                            std::pair<ConstraintGraph::NodeIdType,
                                ElementPose> { freePointId,
                                PointPose {
                                    .position = solvedFreePoint.value(),
                                } },
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
                    merged[freePointId] = PointPose {
                        .position = solvedFreePoint.value(),
                    };
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
                        "[BottomUp][Merge3][C{}] LLP candidate ref={} "
                        "fixedLineA={} fixedLineB={} freePoint={} "
                        "score={}\n",
                        mergeInfo.output.value, referenceIndex,
                        fixedLineAId.value, fixedLineBId.value,
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
            "[BottomUp][Merge3][C{}] Selected LLP candidate score={}\n",
            mergeInfo.output.value, bestScore);
    }

    return bestMergedPose;
}

} // namespace Gcs::Solvers::BottomUp
