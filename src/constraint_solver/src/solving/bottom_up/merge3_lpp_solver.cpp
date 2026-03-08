#include "solving/bottom_up/merge3_lpp_solver.hpp"

// General STD/STL headers
#include <array>
#include <format>
#include <iostream>
#include <limits>
#include <unordered_set>

// Custom headers
#include "solving/bottom_up/merge3_solver_common.hpp"

namespace Gcs::Solvers::BottomUp {

std::optional<ClusterPose> Merge3LppSolver::solve(const Merge3Context& context)
{
    if (context.node.kind != PlanNodeKind::Merge3) {
        return std::nullopt;
    }

    const auto mergeInfo = std::get<Merge3Info>(context.node.info);
    std::optional<ClusterPose> bestMergedPose;
    double bestScore = std::numeric_limits<double>::infinity();

    for (std::size_t referenceIndex = 0; referenceIndex < 3; ++referenceIndex) {
        std::array<std::size_t, 2> baseMovingIndices {};
        std::size_t movingInsertIndex = 0;
        for (std::size_t index = 0; index < 3; ++index) {
            if (index == referenceIndex) {
                continue;
            }
            baseMovingIndices[movingInsertIndex++] = index;
        }

        for (const auto& movingIndices :
            std::array<std::array<std::size_t, 2>, 2> { baseMovingIndices,
                std::array<std::size_t, 2> {
                    baseMovingIndices[1],
                    baseMovingIndices[0],
                } }) {
            const ClusterPose& referenceCluster
                = context.solvedNodePose.at(context.children[referenceIndex]);
            const ClusterPose& pointCluster
                = context.solvedNodePose.at(context.children[movingIndices[0]]);
            const ClusterPose& lineCluster
                = context.solvedNodePose.at(context.children[movingIndices[1]]);

            std::unordered_set<ConstraintGraph::NodeIdType> referenceElements;
            for (const auto& [elementId, /*pose*/ _] : referenceCluster) {
                referenceElements.insert(elementId);
            }

            const auto sharedRefPoints = clusterIntersectionByType(
                context.sourceGraph, referenceCluster, pointCluster, true);
            const auto sharedRefLines = clusterIntersectionByType(
                context.sourceGraph, referenceCluster, lineCluster, false);
            const auto sharedFreePoints = clusterIntersectionByType(
                context.sourceGraph, pointCluster, lineCluster, true);

            for (const auto& fixedPointId : sharedRefPoints) {
                for (const auto& fixedLineId : sharedRefLines) {
                    const auto fixedPointGlobal
                        = getPointPosition(referenceCluster, fixedPointId);
                    const auto fixedLineGlobal
                        = getLinePosition(referenceCluster, fixedLineId);
                    const auto fixedPointCanvas = getPointCanvasPosition(
                        context.sourceGraph, fixedPointId);
                    const auto fixedLineCanvas
                        = getLineCanvasPose(context.sourceGraph, fixedLineId);
                    if (!fixedPointGlobal.has_value()
                        || !fixedLineGlobal.has_value()
                        || !fixedPointCanvas.has_value()
                        || !fixedLineCanvas.has_value()) {
                        continue;
                    }

                    for (const auto& freePointId : sharedFreePoints) {
                        if (referenceElements.contains(freePointId)) {
                            continue;
                        }

                        const auto freePointInPointCluster
                            = getPointPosition(pointCluster, freePointId);
                        const auto fixedPointInPointCluster
                            = getPointPosition(pointCluster, fixedPointId);
                        const auto freePointInLineCluster
                            = getPointPosition(lineCluster, freePointId);
                        const auto fixedLineInLineCluster
                            = getLinePosition(lineCluster, fixedLineId);
                        const auto freePointCanvas = getPointCanvasPosition(
                            context.sourceGraph, freePointId);

                        if (!freePointInPointCluster.has_value()
                            || !fixedPointInPointCluster.has_value()
                            || !freePointInLineCluster.has_value()
                            || !fixedLineInLineCluster.has_value()
                            || !freePointCanvas.has_value()) {
                            continue;
                        }

                        const double distanceToPoint
                            = (freePointInPointCluster.value()
                                - fixedPointInPointCluster.value())
                                  .norm();
                        const double distanceToLine = pointToLineDistanceAbs(
                            freePointInLineCluster.value(),
                            fixedLineInLineCluster.value());

                        const auto solvedFreePoint
                            = solveFreePointFromFixedPointAndLine(
                                fixedPointGlobal.value(),
                                fixedLineGlobal.value(), distanceToPoint,
                                distanceToLine, fixedPointCanvas.value(),
                                fixedLineCanvas.value(),
                                freePointCanvas.value());
                        if (!solvedFreePoint.has_value()) {
                            continue;
                        }

                        const std::array<
                            std::pair<ConstraintGraph::NodeIdType, ElementPose>,
                            2>
                            pointAnchors {
                                std::pair<ConstraintGraph::NodeIdType,
                                    ElementPose> { fixedPointId,
                                    PointPose {
                                        .position = fixedPointGlobal.value(),
                                    } },
                                std::pair<ConstraintGraph::NodeIdType,
                                    ElementPose> { freePointId,
                                    PointPose {
                                        .position = solvedFreePoint.value(),
                                    } },
                            };
                        const std::array<
                            std::pair<ConstraintGraph::NodeIdType, ElementPose>,
                            2>
                            lineAnchors {
                                std::pair<ConstraintGraph::NodeIdType,
                                    ElementPose> {
                                    fixedLineId,
                                    fixedLineGlobal.value(),
                                },
                                std::pair<ConstraintGraph::NodeIdType,
                                    ElementPose> { freePointId,
                                    PointPose {
                                        .position = solvedFreePoint.value(),
                                    } },
                            };

                        const auto transformedPointCluster
                            = transformClusterByAnchors(
                                pointCluster, pointAnchors);
                        const auto transformedLineCluster
                            = transformClusterByAnchors(
                                lineCluster, lineAnchors);
                        if (!transformedPointCluster.has_value()
                            || !transformedLineCluster.has_value()) {
                            continue;
                        }

                        ClusterPose merged = referenceCluster;
                        merged[freePointId] = PointPose {
                            .position = solvedFreePoint.value(),
                        };

                        for (const auto& [elementId, pose] :
                            transformedPointCluster.value()) {
                            if (!merged.contains(elementId)) {
                                merged.emplace(elementId, pose);
                            }
                        }

                        for (const auto& [elementId, pose] :
                            transformedLineCluster.value()) {
                            if (!merged.contains(elementId)) {
                                merged.emplace(elementId, pose);
                            }
                        }

                        const double score
                            = scoreMergedPose(context.sourceGraph, merged);
                        std::cerr << std::format(
                            "[BottomUp][Merge3][C{}] LPP candidate ref={} "
                            "fixedPoint={} fixedLine={} freePoint={} "
                            "score={}\n",
                            mergeInfo.output.value, referenceIndex,
                            fixedPointId.value, fixedLineId.value,
                            freePointId.value, score);

                        if (score < bestScore) {
                            bestScore = score;
                            bestMergedPose = std::move(merged);
                        }
                    }
                }
            }
        }
    }

    if (bestMergedPose.has_value()) {
        std::cerr << std::format(
            "[BottomUp][Merge3][C{}] Selected LPP candidate score={}\n",
            mergeInfo.output.value, bestScore);
    }

    return bestMergedPose;
}

} // namespace Gcs::Solvers::BottomUp
