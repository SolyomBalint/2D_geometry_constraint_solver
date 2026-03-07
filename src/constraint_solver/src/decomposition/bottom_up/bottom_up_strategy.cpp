#include "decomposition/bottom_up/bottom_up_strategy.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <format>
#include <iostream>
#include <limits>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Custom headers
#include "model/constraints.hpp"
#include "model/elements.hpp"
#include "solving/component_solver.hpp"
#include "solving/equations/equation_primitives.hpp"
#include "solving/equations/newton_raphson.hpp"
#include "solving/solvers/heuristics.hpp"

// Thirdparty headers
#include <Eigen/SVD>

namespace Gcs {

namespace {

    constexpr double MIN_LINE_LENGTH = 50.0;
    constexpr double EPSILON = 1e-9;

    struct PointPose {
        Eigen::Vector2d position;
    };

    struct LinePose {
        Eigen::Vector2d p1;
        Eigen::Vector2d p2;
    };

    using ElementPose = std::variant<PointPose, LinePose>;
    using ClusterPose
        = std::unordered_map<ConstraintGraph::NodeIdType, ElementPose>;

    struct RigidTransform {
        Eigen::Matrix2d rotation;
        Eigen::Vector2d translation;
    };

    [[nodiscard]] double safeCanvasLineLength(const Line& line)
    {
        const double canvasLength = (line.canvasP2 - line.canvasP1).norm();
        if (canvasLength < EPSILON) {
            return MIN_LINE_LENGTH;
        }

        return canvasLength;
    }

    [[nodiscard]] std::optional<LinePose> poseAsLine(const ElementPose& pose)
    {
        if (!std::holds_alternative<LinePose>(pose)) {
            return std::nullopt;
        }

        return std::get<LinePose>(pose);
    }

    [[nodiscard]] std::optional<PointPose> poseAsPoint(const ElementPose& pose)
    {
        if (!std::holds_alternative<PointPose>(pose)) {
            return std::nullopt;
        }

        return std::get<PointPose>(pose);
    }

    [[nodiscard]] Eigen::Vector2d lineMidpoint(const LinePose& line)
    {
        return (line.p1 + line.p2) / 2.0;
    }

    [[nodiscard]] std::optional<Eigen::Vector2d> lineUnitDirection(
        const LinePose& line)
    {
        const Eigen::Vector2d direction = line.p2 - line.p1;
        const double length = direction.norm();
        if (length < EPSILON) {
            return std::nullopt;
        }

        return direction / length;
    }

    [[nodiscard]] std::optional<RigidTransform> estimateRigidTransform(
        const std::vector<Eigen::Vector2d>& sourcePoints,
        const std::vector<Eigen::Vector2d>& targetPoints)
    {
        if (sourcePoints.size() != targetPoints.size()
            || sourcePoints.empty()) {
            return std::nullopt;
        }

        if (sourcePoints.size() == 1) {
            return RigidTransform {
                .rotation = Eigen::Matrix2d::Identity(),
                .translation = targetPoints[0] - sourcePoints[0],
            };
        }

        Eigen::Vector2d sourceCentroid = Eigen::Vector2d::Zero();
        Eigen::Vector2d targetCentroid = Eigen::Vector2d::Zero();

        for (std::size_t index = 0; index < sourcePoints.size(); ++index) {
            sourceCentroid += sourcePoints[index];
            targetCentroid += targetPoints[index];
        }

        const double pointCount = static_cast<double>(sourcePoints.size());
        sourceCentroid /= pointCount;
        targetCentroid /= pointCount;

        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        for (std::size_t index = 0; index < sourcePoints.size(); ++index) {
            const Eigen::Vector2d sourceCentered
                = sourcePoints[index] - sourceCentroid;
            const Eigen::Vector2d targetCentered
                = targetPoints[index] - targetCentroid;
            covariance += sourceCentered * targetCentered.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix2d> svd(
            covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix2d rotation = svd.matrixV() * svd.matrixU().transpose();

        if (rotation.determinant() < 0.0) {
            Eigen::Matrix2d matrixV = svd.matrixV();
            matrixV.col(1) *= -1.0;
            rotation = matrixV * svd.matrixU().transpose();
        }

        return RigidTransform {
            .rotation = rotation,
            .translation = targetCentroid - rotation * sourceCentroid,
        };
    }

    [[nodiscard]] ElementPose applyRigidTransform(
        const ElementPose& pose, const RigidTransform& transform)
    {
        if (const auto point = poseAsPoint(pose); point.has_value()) {
            return PointPose {
                .position
                = transform.rotation * point->position + transform.translation,
            };
        }

        const auto line = poseAsLine(pose).value();
        return LinePose {
            .p1 = transform.rotation * line.p1 + transform.translation,
            .p2 = transform.rotation * line.p2 + transform.translation,
        };
    }

    [[nodiscard]] std::optional<ConstraintGraph::EdgeIdType> edgeBetween(
        const ConstraintGraph& graph, ConstraintGraph::NodeIdType first,
        ConstraintGraph::NodeIdType second)
    {
        const auto edge = graph.getGraph().getEdgeBetween(first, second);
        if (!edge.has_value()) {
            return std::nullopt;
        }

        return edge.value();
    }

    [[nodiscard]] std::optional<ClusterPose> solveEdgePrimitive(
        const ConstraintGraph& sourceGraph,
        const std::array<ConstraintGraph::NodeIdType, 2>& elements)
    {
        const auto firstElement = sourceGraph.getElement(elements[0]);
        const auto secondElement = sourceGraph.getElement(elements[1]);
        if (!firstElement || !secondElement) {
            return std::nullopt;
        }

        const auto edgeId = edgeBetween(sourceGraph, elements[0], elements[1]);
        if (!edgeId.has_value() || sourceGraph.isVirtualEdge(edgeId.value())) {
            return std::nullopt;
        }

        const auto constraint
            = sourceGraph.getConstraintForEdge(edgeId.value());
        if (!constraint) {
            return std::nullopt;
        }

        ClusterPose pose;

        if (firstElement->isElementType<Point>()
            && secondElement->isElementType<Point>()
            && constraint->isConstraintType<DistanceConstraint>()) {
            const double distance = constraint->getConstraintValue().value();

            pose.emplace(
                elements[0], PointPose { .position = Eigen::Vector2d::Zero() });
            pose.emplace(elements[1],
                PointPose {
                    .position = Eigen::Vector2d { distance, 0.0 },
                });
            return pose;
        }

        if (constraint->isConstraintType<DistanceConstraint>()
            && ((firstElement->isElementType<Point>()
                    && secondElement->isElementType<Line>())
                || (firstElement->isElementType<Line>()
                    && secondElement->isElementType<Point>()))) {
            const bool firstIsPoint = firstElement->isElementType<Point>();

            const auto& pointElement
                = firstIsPoint ? firstElement : secondElement;
            const auto& lineElement
                = firstIsPoint ? secondElement : firstElement;

            const auto& pointData = pointElement->getElement<Point>();
            const auto& lineData = lineElement->getElement<Line>();

            const double distance = constraint->getConstraintValue().value();
            const double signedCanvasDistance = Solvers::signedDistanceToLine(
                pointData.canvasPosition, lineData.canvasP1, lineData.canvasP2);
            const double sign = (signedCanvasDistance < 0.0) ? -1.0 : 1.0;
            const double y = sign * distance;

            const double lineLength = safeCanvasLineLength(lineData);
            const LinePose solvedLine {
                .p1 = Eigen::Vector2d { -lineLength / 2.0, y },
                .p2 = Eigen::Vector2d { lineLength / 2.0, y },
            };

            const PointPose solvedPoint {
                .position = Eigen::Vector2d::Zero(),
            };

            if (firstIsPoint) {
                pose.emplace(elements[0], solvedPoint);
                pose.emplace(elements[1], solvedLine);
            } else {
                pose.emplace(elements[0], solvedLine);
                pose.emplace(elements[1], solvedPoint);
            }

            return pose;
        }

        if (firstElement->isElementType<Line>()
            && secondElement->isElementType<Line>()
            && constraint->isConstraintType<AngleConstraint>()) {
            const auto& firstLineData = firstElement->getElement<Line>();
            const auto& secondLineData = secondElement->getElement<Line>();
            const auto* angleConstraint
                = constraint->getConstraintAs<AngleConstraint>();
            if (angleConstraint == nullptr) {
                return std::nullopt;
            }

            const double firstLength = safeCanvasLineLength(firstLineData);
            const double secondLength = safeCanvasLineLength(secondLineData);

            Eigen::Vector2d canvasSecondDirection
                = secondLineData.canvasP2 - secondLineData.canvasP1;
            if (angleConstraint->flipOrientation) {
                canvasSecondDirection = -canvasSecondDirection;
            }

            const Eigen::Vector2d canvasFirstDirection
                = firstLineData.canvasP2 - firstLineData.canvasP1;
            const double cross
                = canvasFirstDirection.x() * canvasSecondDirection.y()
                - canvasFirstDirection.y() * canvasSecondDirection.x();

            const double angle = (cross < 0.0) ? -angleConstraint->angle
                                               : angleConstraint->angle;

            const Eigen::Vector2d secondDirection {
                std::cos(angle),
                std::sin(angle),
            };

            pose.emplace(elements[0], LinePose {
                                       .p1 = Eigen::Vector2d {
                                           -firstLength / 2.0,
                                           0.0,
                                       },
                                       .p2 = Eigen::Vector2d {
                                           firstLength / 2.0,
                                           0.0,
                                       },
                                   });
            pose.emplace(elements[1],
                LinePose {
                    .p1 = -secondDirection * secondLength / 2.0,
                    .p2 = secondDirection * secondLength / 2.0,
                });
            return pose;
        }

        return std::nullopt;
    }

    [[nodiscard]] std::optional<ConstraintGraph> buildPrimitiveSubgraph(
        const ConstraintGraph& sourceGraph,
        std::span<const ConstraintGraph::NodeIdType> primitiveElements,
        std::unordered_map<ConstraintGraph::NodeIdType,
            ConstraintGraph::NodeIdType>& sourceToLocalNode)
    {
        ConstraintGraph primitiveGraph;
        sourceToLocalNode.clear();

        for (const auto& sourceNode : primitiveElements) {
            if (!sourceGraph.getGraph().hasNode(sourceNode)) {
                return std::nullopt;
            }

            const auto localNode = primitiveGraph.getGraph().addNode();
            sourceToLocalNode.emplace(sourceNode, localNode);

            const auto sourceElement = sourceGraph.getElement(sourceNode);
            if (!sourceElement) {
                return std::nullopt;
            }

            auto clonedElement = std::make_shared<Element>(*sourceElement);
            (void)primitiveGraph.addElement(
                localNode, std::move(clonedElement));
        }

        for (std::size_t firstIndex = 0; firstIndex < primitiveElements.size();
            ++firstIndex) {
            for (std::size_t secondIndex = firstIndex + 1;
                secondIndex < primitiveElements.size(); ++secondIndex) {
                const auto sourceEdge
                    = edgeBetween(sourceGraph, primitiveElements[firstIndex],
                        primitiveElements[secondIndex]);
                if (!sourceEdge.has_value()) {
                    continue;
                }

                const auto localSource
                    = sourceToLocalNode.at(primitiveElements[firstIndex]);
                const auto localTarget
                    = sourceToLocalNode.at(primitiveElements[secondIndex]);

                if (sourceGraph.isVirtualEdge(sourceEdge.value())) {
                    (void)primitiveGraph.addVirtualEdge(
                        localSource, localTarget);
                    continue;
                }

                const auto localEdge = primitiveGraph.getGraph().addEdge(
                    localSource, localTarget);
                if (!localEdge.has_value()) {
                    return std::nullopt;
                }

                const auto sourceConstraint
                    = sourceGraph.getConstraintForEdge(sourceEdge.value());
                if (!sourceConstraint) {
                    continue;
                }

                auto clonedConstraint
                    = std::make_shared<Constraint>(*sourceConstraint);
                (void)primitiveGraph.addConstraint(
                    localEdge.value(), std::move(clonedConstraint));
            }
        }

        return primitiveGraph;
    }

    [[nodiscard]] std::optional<ClusterPose> solveTrianglePrimitive(
        const ConstraintGraph& sourceGraph,
        const std::array<ConstraintGraph::NodeIdType, 3>& elements)
    {
        std::unordered_map<ConstraintGraph::NodeIdType,
            ConstraintGraph::NodeIdType>
            sourceToLocalNode;
        const auto primitiveGraph
            = buildPrimitiveSubgraph(sourceGraph, elements, sourceToLocalNode);
        if (!primitiveGraph.has_value()) {
            return std::nullopt;
        }

        ConstraintGraph solvedPrimitive = primitiveGraph.value();
        const SolveResult result = classifyAndSolve(solvedPrimitive);
        if (result.status != SolveStatus::Success) {
            return std::nullopt;
        }

        ClusterPose pose;
        for (const auto& sourceNode : elements) {
            const auto localNode = sourceToLocalNode.at(sourceNode);
            const auto solvedElement = solvedPrimitive.getElement(localNode);
            if (!solvedElement) {
                return std::nullopt;
            }

            if (solvedElement->isElementType<Point>()) {
                pose.emplace(sourceNode,
                    PointPose {
                        .position = solvedElement->getElement<Point>().position,
                    });
                continue;
            }

            if (solvedElement->isElementType<Line>()) {
                const auto& line = solvedElement->getElement<Line>();
                pose.emplace(sourceNode,
                    LinePose {
                        .p1 = line.p1,
                        .p2 = line.p2,
                    });
                continue;
            }

            return std::nullopt;
        }

        return pose;
    }

    [[nodiscard]] std::optional<ClusterPose> mergeChildClusterIntoReference(
        ClusterPose referenceCluster, const ClusterPose& movingCluster)
    {
        std::vector<Eigen::Vector2d> sourcePoints;
        std::vector<Eigen::Vector2d> targetPoints;

        for (const auto& [elementId, movingPose] : movingCluster) {
            if (!referenceCluster.contains(elementId)) {
                continue;
            }

            const auto& referencePose = referenceCluster.at(elementId);

            if (const auto movingPoint = poseAsPoint(movingPose);
                movingPoint.has_value()) {
                const auto referencePoint = poseAsPoint(referencePose);
                if (!referencePoint.has_value()) {
                    return std::nullopt;
                }

                sourcePoints.push_back(movingPoint->position);
                targetPoints.push_back(referencePoint->position);
                continue;
            }

            const auto movingLine = poseAsLine(movingPose);
            const auto referenceLine = poseAsLine(referencePose);
            if (!movingLine.has_value() || !referenceLine.has_value()) {
                return std::nullopt;
            }

            const auto movingDirection = lineUnitDirection(movingLine.value());
            const auto referenceDirection
                = lineUnitDirection(referenceLine.value());
            if (!movingDirection.has_value()
                || !referenceDirection.has_value()) {
                return std::nullopt;
            }

            const Eigen::Vector2d movingCenter
                = lineMidpoint(movingLine.value());
            const Eigen::Vector2d referenceCenter
                = lineMidpoint(referenceLine.value());

            sourcePoints.push_back(movingCenter);
            targetPoints.push_back(referenceCenter);
            sourcePoints.push_back(movingCenter + movingDirection.value());
            targetPoints.push_back(
                referenceCenter + referenceDirection.value());
        }

        const auto transform
            = estimateRigidTransform(sourcePoints, targetPoints);
        if (!transform.has_value()) {
            return std::nullopt;
        }

        for (const auto& [elementId, movingPose] : movingCluster) {
            if (referenceCluster.contains(elementId)) {
                continue;
            }

            referenceCluster.emplace(
                elementId, applyRigidTransform(movingPose, transform.value()));
        }

        return referenceCluster;
    }

    [[nodiscard]] std::optional<Eigen::Vector2d> getPointPosition(
        const ClusterPose& cluster, ConstraintGraph::NodeIdType elementId)
    {
        if (!cluster.contains(elementId)) {
            return std::nullopt;
        }

        const auto point = poseAsPoint(cluster.at(elementId));
        if (!point.has_value()) {
            return std::nullopt;
        }

        return point->position;
    }

    [[nodiscard]] std::optional<Eigen::Vector2d> getPointCanvasPosition(
        const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId)
    {
        const auto element = graph.getElement(elementId);
        if (!element || !element->isElementType<Point>()) {
            return std::nullopt;
        }

        return element->getElement<Point>().canvasPosition;
    }

    [[nodiscard]] std::optional<LinePose> getLinePosition(
        const ClusterPose& cluster, ConstraintGraph::NodeIdType elementId)
    {
        if (!cluster.contains(elementId)) {
            return std::nullopt;
        }

        return poseAsLine(cluster.at(elementId));
    }

    [[nodiscard]] std::optional<LinePose> getLineCanvasPose(
        const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId)
    {
        const auto element = graph.getElement(elementId);
        if (!element || !element->isElementType<Line>()) {
            return std::nullopt;
        }

        const auto& line = element->getElement<Line>();
        return LinePose {
            .p1 = line.canvasP1,
            .p2 = line.canvasP2,
        };
    }

    [[nodiscard]] bool isPointElement(
        const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId)
    {
        const auto element = graph.getElement(elementId);
        return element && element->isElementType<Point>();
    }

    [[nodiscard]] bool isLineElement(
        const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId)
    {
        const auto element = graph.getElement(elementId);
        return element && element->isElementType<Line>();
    }

    [[nodiscard]] std::vector<ConstraintGraph::NodeIdType>
    clusterIntersectionByType(const ConstraintGraph& graph,
        const ClusterPose& first, const ClusterPose& second, bool selectPoints)
    {
        std::vector<ConstraintGraph::NodeIdType> intersection;
        for (const auto& [elementId, /*pose*/ _] : first) {
            if (!second.contains(elementId)) {
                continue;
            }

            if (selectPoints && !isPointElement(graph, elementId)) {
                continue;
            }

            if (!selectPoints && !isLineElement(graph, elementId)) {
                continue;
            }

            intersection.push_back(elementId);
        }

        std::ranges::sort(intersection);
        auto uniqueBegin = std::ranges::unique(intersection).begin();
        intersection.erase(uniqueBegin, intersection.end());
        return intersection;
    }

    [[nodiscard]] double signForDistance(double value)
    {
        return (value > 0.0) ? 1.0 : -1.0;
    }

    [[nodiscard]] double lineLength(const LinePose& line)
    {
        const double length = (line.p2 - line.p1).norm();
        return (length < EPSILON) ? MIN_LINE_LENGTH : length;
    }

    [[nodiscard]] std::pair<Eigen::Vector2d, Eigen::Vector2d>
    reconstructLineEndpoints(const Eigen::Vector2d& referencePoint1,
        const Eigen::Vector2d& referencePoint2, double normalX, double normalY,
        double perpendicularOffset, double canvasLineLength)
    {
        const Eigen::Vector2d normal { normalX, normalY };

        auto projectOntoLine
            = [&normal, perpendicularOffset](const Eigen::Vector2d& point) {
                  const double signedDistance
                      = normal.dot(point) - perpendicularOffset;
                  return point - signedDistance * normal;
              };

        const Eigen::Vector2d projection1 = projectOntoLine(referencePoint1);
        const Eigen::Vector2d projection2 = projectOntoLine(referencePoint2);

        const Eigen::Vector2d lineDirection { -normalY, normalX };
        const Eigen::Vector2d midpoint = (projection1 + projection2) / 2.0;
        const double projectionSpan
            = std::abs(lineDirection.dot(projection2 - projection1));
        const double halfLength
            = std::max(canvasLineLength, projectionSpan) / 2.0;

        return { midpoint - halfLength * lineDirection,
            midpoint + halfLength * lineDirection };
    }

    [[nodiscard]] double pointToLineDistanceAbs(
        const Eigen::Vector2d& point, const LinePose& line)
    {
        return std::abs(Solvers::signedDistanceToLine(point, line.p1, line.p2));
    }

    [[nodiscard]] std::optional<ClusterPose> transformClusterByTwoPointAnchors(
        const ClusterPose& movingCluster,
        ConstraintGraph::NodeIdType fixedPoint,
        ConstraintGraph::NodeIdType freePoint,
        const Eigen::Vector2d& fixedPointGlobal,
        const Eigen::Vector2d& freePointGlobal)
    {
        const auto fixedLocal = getPointPosition(movingCluster, fixedPoint);
        const auto freeLocal = getPointPosition(movingCluster, freePoint);
        if (!fixedLocal.has_value() || !freeLocal.has_value()) {
            return std::nullopt;
        }

        const std::vector<Eigen::Vector2d> sourcePoints {
            fixedLocal.value(),
            freeLocal.value(),
        };
        const std::vector<Eigen::Vector2d> targetPoints {
            fixedPointGlobal,
            freePointGlobal,
        };

        const auto transform
            = estimateRigidTransform(sourcePoints, targetPoints);
        if (!transform.has_value()) {
            return std::nullopt;
        }

        ClusterPose transformed;
        transformed.reserve(movingCluster.size());
        for (const auto& [elementId, pose] : movingCluster) {
            transformed.emplace(
                elementId, applyRigidTransform(pose, transform.value()));
        }

        return transformed;
    }

    [[nodiscard]] std::optional<ClusterPose> transformClusterByAnchors(
        const ClusterPose& movingCluster,
        std::span<const std::pair<ConstraintGraph::NodeIdType, ElementPose>>
            anchors)
    {
        std::vector<Eigen::Vector2d> sourcePoints;
        std::vector<Eigen::Vector2d> targetPoints;

        for (const auto& [elementId, targetPose] : anchors) {
            if (!movingCluster.contains(elementId)) {
                return std::nullopt;
            }

            const auto& sourcePose = movingCluster.at(elementId);
            if (const auto targetPoint = poseAsPoint(targetPose);
                targetPoint.has_value()) {
                const auto sourcePoint = poseAsPoint(sourcePose);
                if (!sourcePoint.has_value()) {
                    return std::nullopt;
                }

                sourcePoints.push_back(sourcePoint->position);
                targetPoints.push_back(targetPoint->position);
                continue;
            }

            const auto targetLine = poseAsLine(targetPose);
            const auto sourceLine = poseAsLine(sourcePose);
            if (!targetLine.has_value() || !sourceLine.has_value()) {
                return std::nullopt;
            }

            const auto sourceDirection = lineUnitDirection(sourceLine.value());
            const auto targetDirection = lineUnitDirection(targetLine.value());
            if (!sourceDirection.has_value() || !targetDirection.has_value()) {
                return std::nullopt;
            }

            const Eigen::Vector2d sourceCenter
                = lineMidpoint(sourceLine.value());
            const Eigen::Vector2d targetCenter
                = lineMidpoint(targetLine.value());
            sourcePoints.push_back(sourceCenter);
            targetPoints.push_back(targetCenter);
            sourcePoints.push_back(sourceCenter + sourceDirection.value());
            targetPoints.push_back(targetCenter + targetDirection.value());
        }

        const auto transform
            = estimateRigidTransform(sourcePoints, targetPoints);
        if (!transform.has_value()) {
            return std::nullopt;
        }

        ClusterPose transformed;
        transformed.reserve(movingCluster.size());
        for (const auto& [elementId, pose] : movingCluster) {
            transformed.emplace(
                elementId, applyRigidTransform(pose, transform.value()));
        }

        return transformed;
    }

    [[nodiscard]] double scoreMergedPose(
        const ConstraintGraph& sourceGraph, const ClusterPose& mergedPose)
    {
        double score = 0.0;
        std::size_t scoreTerms = 0;

        for (const auto& [elementId, pose] : mergedPose) {
            if (const auto pointPose = poseAsPoint(pose);
                pointPose.has_value()) {
                const auto canvasPoint
                    = getPointCanvasPosition(sourceGraph, elementId);
                if (!canvasPoint.has_value()) {
                    continue;
                }

                score += (pointPose->position - canvasPoint.value())
                             .squaredNorm();
                ++scoreTerms;
                continue;
            }

            const auto linePose = poseAsLine(pose);
            const auto canvasLine = getLineCanvasPose(sourceGraph, elementId);
            if (!linePose.has_value() || !canvasLine.has_value()) {
                continue;
            }

            const Eigen::Vector2d solverMid = lineMidpoint(linePose.value());
            const Eigen::Vector2d canvasMid = lineMidpoint(canvasLine.value());
            score += (solverMid - canvasMid).squaredNorm();

            const auto solverDirection = lineUnitDirection(linePose.value());
            const auto canvasDirection = lineUnitDirection(canvasLine.value());
            if (solverDirection.has_value() && canvasDirection.has_value()) {
                const double alignment
                    = std::abs(solverDirection->dot(canvasDirection.value()));
                score += (1.0 - alignment) * 100.0;
            }

            ++scoreTerms;
        }

        if (scoreTerms == 0) {
            return std::numeric_limits<double>::infinity();
        }

        return score;
    }

    [[nodiscard]] std::optional<LinePose> solveFreeLineFromFixedPoints(
        const Eigen::Vector2d& fixedPointA, const Eigen::Vector2d& fixedPointB,
        double distanceA, double distanceB, const Eigen::Vector2d& canvasPointA,
        const Eigen::Vector2d& canvasPointB, const LinePose& canvasFreeLine)
    {
        const double canvasSignedA = Solvers::signedDistanceToLine(
            canvasPointA, canvasFreeLine.p1, canvasFreeLine.p2);
        const double canvasSignedB = Solvers::signedDistanceToLine(
            canvasPointB, canvasFreeLine.p1, canvasFreeLine.p2);
        const double signedDistA = signForDistance(canvasSignedA) * distanceA;
        const double signedDistB = signForDistance(canvasSignedB) * distanceB;

        const Eigen::Vector2d delta = fixedPointB - fixedPointA;
        auto normalDistanceEquation = Equations::lineNormalSignedDistanceDiff(
            delta.x(), delta.y(), signedDistA, signedDistB);
        auto unitConstraintEquation = Equations::unitNormalConstraint();

        Eigen::Vector2d canvasDirection = canvasFreeLine.p2 - canvasFreeLine.p1;
        if (canvasDirection.norm() < EPSILON) {
            canvasDirection = Eigen::Vector2d { 1.0, 0.0 };
        }

        const Eigen::Vector2d canvasNormal {
            -canvasDirection.y() / canvasDirection.norm(),
            canvasDirection.x() / canvasDirection.norm(),
        };

        const std::array<Eigen::Vector2d, 2> initialGuesses {
            canvasNormal,
            -canvasNormal,
        };
        const auto candidateNormals = Equations::solve2D(
            normalDistanceEquation, unitConstraintEquation, initialGuesses);

        const double offset0
            = candidateNormals[0].dot(fixedPointA) - signedDistA;
        const double offset1
            = candidateNormals[1].dot(fixedPointA) - signedDistA;

        const auto [normalX, normalY, offset]
            = Solvers::pickLineBySignedDistances(canvasSignedA, canvasSignedB,
                candidateNormals[0], candidateNormals[1], fixedPointA,
                fixedPointB, offset0, offset1);

        const auto [lineP1, lineP2] = reconstructLineEndpoints(fixedPointA,
            fixedPointB, normalX, normalY, offset, lineLength(canvasFreeLine));

        return LinePose {
            .p1 = lineP1,
            .p2 = lineP2,
        };
    }

    [[nodiscard]] std::optional<Eigen::Vector2d>
    solveFreePointFromFixedPointAndLine(const Eigen::Vector2d& fixedPoint,
        const LinePose& fixedLine, double distanceToPoint,
        double distanceToLine, const Eigen::Vector2d& canvasFixedPoint,
        const LinePose& canvasFixedLine, const Eigen::Vector2d& canvasFreePoint)
    {
        const double canvasSigned = Solvers::signedDistanceToLine(
            canvasFreePoint, canvasFixedLine.p1, canvasFixedLine.p2);
        const double signedDistance
            = signForDistance(canvasSigned) * distanceToLine;

        auto pointEquation = Equations::pointToPointDistance(
            fixedPoint.x(), fixedPoint.y(), distanceToPoint);
        auto lineEquation = Equations::pointToLineDistance(fixedLine.p1.x(),
            fixedLine.p1.y(), fixedLine.p2.x(), fixedLine.p2.y(),
            signedDistance, lineLength(fixedLine));

        const auto candidates = Equations::solve2D(pointEquation, lineEquation);

        const Eigen::Vector2d solverFoot = Solvers::perpendicularFoot(
            fixedPoint, fixedLine.p1, fixedLine.p2);
        const Eigen::Vector2d canvasFoot = Solvers::perpendicularFoot(
            canvasFixedPoint, canvasFixedLine.p1, canvasFixedLine.p2);

        return Solvers::pickByTriangleOrientationWithFallback(canvasFixedPoint,
            canvasFoot, canvasFreePoint, fixedPoint, solverFoot, candidates[0],
            candidates[1]);
    }

    [[nodiscard]] std::optional<Eigen::Vector2d> solveFreePointFromFixedLines(
        const LinePose& fixedLineA, const LinePose& fixedLineB,
        double distanceToLineA, double distanceToLineB,
        const LinePose& canvasLineA, const LinePose& canvasLineB,
        const Eigen::Vector2d& canvasFreePoint)
    {
        const double canvasSignedA = Solvers::signedDistanceToLine(
            canvasFreePoint, canvasLineA.p1, canvasLineA.p2);
        const double canvasSignedB = Solvers::signedDistanceToLine(
            canvasFreePoint, canvasLineB.p1, canvasLineB.p2);
        const double signedDistanceA
            = signForDistance(canvasSignedA) * distanceToLineA;
        const double signedDistanceB
            = signForDistance(canvasSignedB) * distanceToLineB;

        auto eqA = Equations::pointToLineDistance(fixedLineA.p1.x(),
            fixedLineA.p1.y(), fixedLineA.p2.x(), fixedLineA.p2.y(),
            signedDistanceA, lineLength(fixedLineA));
        auto eqB = Equations::pointToLineDistance(fixedLineB.p1.x(),
            fixedLineB.p1.y(), fixedLineB.p2.x(), fixedLineB.p2.y(),
            signedDistanceB, lineLength(fixedLineB));

        const auto candidates = Equations::solve2D(eqA, eqB);

        const auto solverIntersection = Solvers::lineLineIntersection(
            fixedLineA.p1, fixedLineA.p2, fixedLineB.p1, fixedLineB.p2);
        const auto canvasIntersection = Solvers::lineLineIntersection(
            canvasLineA.p1, canvasLineA.p2, canvasLineB.p1, canvasLineB.p2);

        if (solverIntersection.has_value() && canvasIntersection.has_value()) {
            const auto lineADirection = lineUnitDirection(fixedLineA);
            const auto canvasDirection = lineUnitDirection(canvasLineA);
            if (!lineADirection.has_value() || !canvasDirection.has_value()) {
                return std::nullopt;
            }

            const Eigen::Vector2d solverRefPoint
                = solverIntersection.value() + lineADirection.value();
            const Eigen::Vector2d canvasRefPoint
                = canvasIntersection.value() + canvasDirection.value();

            return Solvers::pickByTriangleOrientationWithFallback(
                canvasIntersection.value(), canvasRefPoint, canvasFreePoint,
                solverIntersection.value(), solverRefPoint, candidates[0],
                candidates[1]);
        }

        const double dist0 = (candidates[0] - canvasFreePoint).squaredNorm();
        const double dist1 = (candidates[1] - canvasFreePoint).squaredNorm();
        return (dist0 <= dist1) ? candidates[0] : candidates[1];
    }

    [[nodiscard]] std::optional<ClusterPose> trySolveMerge3Ppp(
        const ConstraintGraph& sourceGraph, const PlanNode& node,
        std::span<const MathUtils::GeneralTreeNodeId> children,
        const std::unordered_map<MathUtils::GeneralTreeNodeId, ClusterPose>&
            solvedNodePose)
    {
        if (node.kind != PlanNodeKind::Merge3) {
            return std::nullopt;
        }

        const auto mergeInfo = std::get<Merge3Info>(node.info);
        auto isPointElement
            = [&sourceGraph](ConstraintGraph::NodeIdType elementId) {
                  const auto element = sourceGraph.getElement(elementId);
                  return element && element->isElementType<Point>();
              };

        auto clusterPointIntersection = [&isPointElement](
                                            const ClusterPose& first,
                                            const ClusterPose& second) {
            std::vector<ConstraintGraph::NodeIdType> intersection;
            for (const auto& [elementId, /*pose*/ _] : first) {
                if (second.contains(elementId) && isPointElement(elementId)) {
                    intersection.push_back(elementId);
                }
            }

            std::ranges::sort(intersection);
            auto uniqueBegin = std::ranges::unique(intersection).begin();
            intersection.erase(uniqueBegin, intersection.end());
            return intersection;
        };

        auto scoreMergedPose = [&sourceGraph](const ClusterPose& mergedPose) {
            double score = 0.0;
            std::size_t usedPointCount = 0;

            for (const auto& [elementId, pose] : mergedPose) {
                const auto pointPose = poseAsPoint(pose);
                if (!pointPose.has_value()) {
                    continue;
                }

                const auto canvasPoint
                    = getPointCanvasPosition(sourceGraph, elementId);
                if (!canvasPoint.has_value()) {
                    continue;
                }

                score += (pointPose->position - canvasPoint.value())
                             .squaredNorm();
                ++usedPointCount;
            }

            if (usedPointCount == 0) {
                return std::numeric_limits<double>::infinity();
            }

            return score;
        };

        std::optional<ClusterPose> bestMergedPose;
        double bestScore = std::numeric_limits<double>::infinity();
        int attemptedCandidates = 0;

        std::cerr << std::format(
            "[BottomUp][Merge3][C{}] Trying pairwise-anchor PPP merge\n",
            mergeInfo.output.value);

        for (std::size_t referenceIndex = 0; referenceIndex < 3;
            ++referenceIndex) {
            std::array<std::size_t, 2> movingIndices {};
            std::size_t movingInsertIndex = 0;
            for (std::size_t index = 0; index < 3; ++index) {
                if (index == referenceIndex) {
                    continue;
                }
                movingIndices[movingInsertIndex++] = index;
            }

            const ClusterPose& referenceCluster
                = solvedNodePose.at(children[referenceIndex]);
            const ClusterPose& movingClusterA
                = solvedNodePose.at(children[movingIndices[0]]);
            const ClusterPose& movingClusterB
                = solvedNodePose.at(children[movingIndices[1]]);

            std::unordered_set<ConstraintGraph::NodeIdType> referenceElements;
            referenceElements.reserve(referenceCluster.size());
            for (const auto& [elementId, /*pose*/ _] : referenceCluster) {
                referenceElements.insert(elementId);
            }

            const auto sharedRefA
                = clusterPointIntersection(referenceCluster, movingClusterA);
            const auto sharedRefB
                = clusterPointIntersection(referenceCluster, movingClusterB);
            const auto sharedAB
                = clusterPointIntersection(movingClusterA, movingClusterB);

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
                        if (distanceAFree < EPSILON
                            || distanceBFree < EPSILON) {
                            continue;
                        }

                        const auto fixedACanvas
                            = getPointCanvasPosition(sourceGraph, fixedPointA);
                        const auto fixedBCanvas
                            = getPointCanvasPosition(sourceGraph, fixedPointB);
                        const auto freeCanvas
                            = getPointCanvasPosition(sourceGraph, freePointId);
                        if (!fixedACanvas.has_value()
                            || !fixedBCanvas.has_value()
                            || !freeCanvas.has_value()) {
                            continue;
                        }

                        auto distanceFromAFunction
                            = Equations::pointToPointDistance(
                                fixedAInGlobal->x(), fixedAInGlobal->y(),
                                distanceAFree);
                        auto distanceFromBFunction
                            = Equations::pointToPointDistance(
                                fixedBInGlobal->x(), fixedBInGlobal->y(),
                                distanceBFree);

                        const auto possibleFreePositions = Equations::solve2D(
                            distanceFromAFunction, distanceFromBFunction);

                        const Eigen::Vector2d selectedFreePoint
                            = Solvers::pickByTriangleOrientation(
                                fixedACanvas.value(), fixedBCanvas.value(),
                                freeCanvas.value(), fixedAInGlobal.value(),
                                fixedBInGlobal.value(),
                                possibleFreePositions[0],
                                possibleFreePositions[1]);

                        const auto transformedA
                            = transformClusterByTwoPointAnchors(movingClusterA,
                                fixedPointA, freePointId,
                                fixedAInGlobal.value(), selectedFreePoint);
                        const auto transformedB
                            = transformClusterByTwoPointAnchors(movingClusterB,
                                fixedPointB, freePointId,
                                fixedBInGlobal.value(), selectedFreePoint);
                        if (!transformedA.has_value()
                            || !transformedB.has_value()) {
                            continue;
                        }

                        ClusterPose merged = referenceCluster;
                        merged[freePointId]
                            = PointPose { .position = selectedFreePoint };

                        for (const auto& [elementId, pose] :
                            transformedA.value()) {
                            if (!merged.contains(elementId)) {
                                merged.emplace(elementId, pose);
                            }
                        }
                        for (const auto& [elementId, pose] :
                            transformedB.value()) {
                            if (!merged.contains(elementId)) {
                                merged.emplace(elementId, pose);
                            }
                        }

                        ++attemptedCandidates;
                        const double score = scoreMergedPose(merged);
                        std::cerr << std::format(
                            "[BottomUp][Merge3][C{}] candidate #{} "
                            "ref={} fixedA={} fixedB={} free={} score={}\n",
                            mergeInfo.output.value, attemptedCandidates,
                            referenceIndex, fixedPointA.value,
                            fixedPointB.value, freePointId.value, score);

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

    [[nodiscard]] std::optional<ClusterPose> trySolveMerge3Pll(
        const ConstraintGraph& sourceGraph, const PlanNode& node,
        std::span<const MathUtils::GeneralTreeNodeId> children,
        const std::unordered_map<MathUtils::GeneralTreeNodeId, ClusterPose>&
            solvedNodePose)
    {
        if (node.kind != PlanNodeKind::Merge3) {
            return std::nullopt;
        }

        const auto mergeInfo = std::get<Merge3Info>(node.info);
        std::optional<ClusterPose> bestMergedPose;
        double bestScore = std::numeric_limits<double>::infinity();

        for (std::size_t referenceIndex = 0; referenceIndex < 3;
            ++referenceIndex) {
            std::array<std::size_t, 2> movingIndices {};
            std::size_t movingInsertIndex = 0;
            for (std::size_t index = 0; index < 3; ++index) {
                if (index == referenceIndex) {
                    continue;
                }
                movingIndices[movingInsertIndex++] = index;
            }

            const ClusterPose& referenceCluster
                = solvedNodePose.at(children[referenceIndex]);
            const ClusterPose& movingClusterA
                = solvedNodePose.at(children[movingIndices[0]]);
            const ClusterPose& movingClusterB
                = solvedNodePose.at(children[movingIndices[1]]);

            std::unordered_set<ConstraintGraph::NodeIdType> referenceElements;
            for (const auto& [elementId, /*pose*/ _] : referenceCluster) {
                referenceElements.insert(elementId);
            }

            const auto sharedRefAPoints = clusterIntersectionByType(
                sourceGraph, referenceCluster, movingClusterA, true);
            const auto sharedRefBPoints = clusterIntersectionByType(
                sourceGraph, referenceCluster, movingClusterB, true);
            const auto sharedABLines = clusterIntersectionByType(
                sourceGraph, movingClusterA, movingClusterB, false);

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
                        = getPointCanvasPosition(sourceGraph, fixedPointA);
                    const auto fixedBCanvas
                        = getPointCanvasPosition(sourceGraph, fixedPointB);
                    if (!fixedAInGlobal.has_value()
                        || !fixedBInGlobal.has_value()
                        || !fixedACanvas.has_value()
                        || !fixedBCanvas.has_value()) {
                        continue;
                    }

                    for (const auto& freeLineId : freeLineCandidates) {
                        const auto freeLineCanvas
                            = getLineCanvasPose(sourceGraph, freeLineId);
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

                        const auto solvedFreeLine
                            = solveFreeLineFromFixedPoints(
                                fixedAInGlobal.value(), fixedBInGlobal.value(),
                                distanceA, distanceB, fixedACanvas.value(),
                                fixedBCanvas.value(), freeLineCanvas.value());
                        if (!solvedFreeLine.has_value()) {
                            continue;
                        }

                        const std::array<
                            std::pair<ConstraintGraph::NodeIdType, ElementPose>,
                            2>
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
                            std::pair<ConstraintGraph::NodeIdType, ElementPose>,
                            2>
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

                        const auto transformedA = transformClusterByAnchors(
                            movingClusterA, anchorsA);
                        const auto transformedB = transformClusterByAnchors(
                            movingClusterB, anchorsB);
                        if (!transformedA.has_value()
                            || !transformedB.has_value()) {
                            continue;
                        }

                        ClusterPose merged = referenceCluster;
                        merged[freeLineId] = solvedFreeLine.value();

                        for (const auto& [elementId, pose] :
                            transformedA.value()) {
                            if (!merged.contains(elementId)) {
                                merged.emplace(elementId, pose);
                            }
                        }
                        for (const auto& [elementId, pose] :
                            transformedB.value()) {
                            if (!merged.contains(elementId)) {
                                merged.emplace(elementId, pose);
                            }
                        }

                        const double score
                            = scoreMergedPose(sourceGraph, merged);
                        std::cerr << std::format(
                            "[BottomUp][Merge3][C{}] PLL candidate ref={} "
                            "fixedA={} fixedB={} freeLine={} score={}\n",
                            mergeInfo.output.value, referenceIndex,
                            fixedPointA.value, fixedPointB.value,
                            freeLineId.value, score);

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

    [[nodiscard]] std::optional<ClusterPose> trySolveMerge3Lpp(
        const ConstraintGraph& sourceGraph, const PlanNode& node,
        std::span<const MathUtils::GeneralTreeNodeId> children,
        const std::unordered_map<MathUtils::GeneralTreeNodeId, ClusterPose>&
            solvedNodePose)
    {
        if (node.kind != PlanNodeKind::Merge3) {
            return std::nullopt;
        }

        const auto mergeInfo = std::get<Merge3Info>(node.info);
        std::optional<ClusterPose> bestMergedPose;
        double bestScore = std::numeric_limits<double>::infinity();

        for (std::size_t referenceIndex = 0; referenceIndex < 3;
            ++referenceIndex) {
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
                    = solvedNodePose.at(children[referenceIndex]);
                const ClusterPose& pointCluster
                    = solvedNodePose.at(children[movingIndices[0]]);
                const ClusterPose& lineCluster
                    = solvedNodePose.at(children[movingIndices[1]]);

                std::unordered_set<ConstraintGraph::NodeIdType>
                    referenceElements;
                for (const auto& [elementId, /*pose*/ _] : referenceCluster) {
                    referenceElements.insert(elementId);
                }

                const auto sharedRefPoints = clusterIntersectionByType(
                    sourceGraph, referenceCluster, pointCluster, true);
                const auto sharedRefLines = clusterIntersectionByType(
                    sourceGraph, referenceCluster, lineCluster, false);
                const auto sharedFreePoints = clusterIntersectionByType(
                    sourceGraph, pointCluster, lineCluster, true);

                for (const auto& fixedPointId : sharedRefPoints) {
                    for (const auto& fixedLineId : sharedRefLines) {
                        const auto fixedPointGlobal
                            = getPointPosition(referenceCluster, fixedPointId);
                        const auto fixedLineGlobal
                            = getLinePosition(referenceCluster, fixedLineId);
                        const auto fixedPointCanvas
                            = getPointCanvasPosition(sourceGraph, fixedPointId);
                        const auto fixedLineCanvas
                            = getLineCanvasPose(sourceGraph, fixedLineId);
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
                                sourceGraph, freePointId);

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
                            const double distanceToLine
                                = pointToLineDistanceAbs(
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
                                std::pair<ConstraintGraph::NodeIdType,
                                    ElementPose>,
                                2>
                                pointAnchors {
                                    std::pair<ConstraintGraph::NodeIdType,
                                        ElementPose> { fixedPointId,
                                        PointPose {
                                            .position
                                            = fixedPointGlobal.value(),
                                        } },
                                    std::pair<ConstraintGraph::NodeIdType,
                                        ElementPose> { freePointId,
                                        PointPose {
                                            .position = solvedFreePoint.value(),
                                        } },
                                };
                            const std::array<
                                std::pair<ConstraintGraph::NodeIdType,
                                    ElementPose>,
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
                                = scoreMergedPose(sourceGraph, merged);
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

    [[nodiscard]] std::optional<ClusterPose> trySolveMerge3Llp(
        const ConstraintGraph& sourceGraph, const PlanNode& node,
        std::span<const MathUtils::GeneralTreeNodeId> children,
        const std::unordered_map<MathUtils::GeneralTreeNodeId, ClusterPose>&
            solvedNodePose)
    {
        if (node.kind != PlanNodeKind::Merge3) {
            return std::nullopt;
        }

        const auto mergeInfo = std::get<Merge3Info>(node.info);
        std::optional<ClusterPose> bestMergedPose;
        double bestScore = std::numeric_limits<double>::infinity();

        for (std::size_t referenceIndex = 0; referenceIndex < 3;
            ++referenceIndex) {
            std::array<std::size_t, 2> movingIndices {};
            std::size_t movingInsertIndex = 0;
            for (std::size_t index = 0; index < 3; ++index) {
                if (index == referenceIndex) {
                    continue;
                }
                movingIndices[movingInsertIndex++] = index;
            }

            const ClusterPose& referenceCluster
                = solvedNodePose.at(children[referenceIndex]);
            const ClusterPose& movingClusterA
                = solvedNodePose.at(children[movingIndices[0]]);
            const ClusterPose& movingClusterB
                = solvedNodePose.at(children[movingIndices[1]]);

            std::unordered_set<ConstraintGraph::NodeIdType> referenceElements;
            for (const auto& [elementId, /*pose*/ _] : referenceCluster) {
                referenceElements.insert(elementId);
            }

            const auto sharedRefALines = clusterIntersectionByType(
                sourceGraph, referenceCluster, movingClusterA, false);
            const auto sharedRefBLines = clusterIntersectionByType(
                sourceGraph, referenceCluster, movingClusterB, false);
            const auto sharedABPoints = clusterIntersectionByType(
                sourceGraph, movingClusterA, movingClusterB, true);

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
                        = getLineCanvasPose(sourceGraph, fixedLineAId);
                    const auto fixedLineBCanvas
                        = getLineCanvasPose(sourceGraph, fixedLineBId);
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
                        const auto freePointCanvas
                            = getPointCanvasPosition(sourceGraph, freePointId);

                        if (!freePointInA.has_value()
                            || !freePointInB.has_value()
                            || !fixedLineAInA.has_value()
                            || !fixedLineBInB.has_value()
                            || !freePointCanvas.has_value()) {
                            continue;
                        }

                        const double distanceToA = pointToLineDistanceAbs(
                            freePointInA.value(), fixedLineAInA.value());
                        const double distanceToB = pointToLineDistanceAbs(
                            freePointInB.value(), fixedLineBInB.value());

                        const auto solvedFreePoint
                            = solveFreePointFromFixedLines(
                                fixedLineAGlobal.value(),
                                fixedLineBGlobal.value(), distanceToA,
                                distanceToB, fixedLineACanvas.value(),
                                fixedLineBCanvas.value(),
                                freePointCanvas.value());
                        if (!solvedFreePoint.has_value()) {
                            continue;
                        }

                        const std::array<
                            std::pair<ConstraintGraph::NodeIdType, ElementPose>,
                            2>
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
                            std::pair<ConstraintGraph::NodeIdType, ElementPose>,
                            2>
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

                        const auto transformedA = transformClusterByAnchors(
                            movingClusterA, anchorsA);
                        const auto transformedB = transformClusterByAnchors(
                            movingClusterB, anchorsB);
                        if (!transformedA.has_value()
                            || !transformedB.has_value()) {
                            continue;
                        }

                        ClusterPose merged = referenceCluster;
                        merged[freePointId] = PointPose {
                            .position = solvedFreePoint.value(),
                        };
                        for (const auto& [elementId, pose] :
                            transformedA.value()) {
                            if (!merged.contains(elementId)) {
                                merged.emplace(elementId, pose);
                            }
                        }
                        for (const auto& [elementId, pose] :
                            transformedB.value()) {
                            if (!merged.contains(elementId)) {
                                merged.emplace(elementId, pose);
                            }
                        }

                        const double score
                            = scoreMergedPose(sourceGraph, merged);
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

    [[nodiscard]] bool detectUnsolvableMerge3Lll(
        const ConstraintGraph& sourceGraph, const PlanNode& node,
        std::span<const MathUtils::GeneralTreeNodeId> children,
        const std::unordered_map<MathUtils::GeneralTreeNodeId, ClusterPose>&
            solvedNodePose)
    {
        if (node.kind != PlanNodeKind::Merge3) {
            return false;
        }

        for (std::size_t referenceIndex = 0; referenceIndex < 3;
            ++referenceIndex) {
            std::array<std::size_t, 2> movingIndices {};
            std::size_t movingInsertIndex = 0;
            for (std::size_t index = 0; index < 3; ++index) {
                if (index == referenceIndex) {
                    continue;
                }
                movingIndices[movingInsertIndex++] = index;
            }

            const ClusterPose& referenceCluster
                = solvedNodePose.at(children[referenceIndex]);
            const ClusterPose& movingClusterA
                = solvedNodePose.at(children[movingIndices[0]]);
            const ClusterPose& movingClusterB
                = solvedNodePose.at(children[movingIndices[1]]);

            std::unordered_set<ConstraintGraph::NodeIdType> referenceElements;
            for (const auto& [elementId, /*pose*/ _] : referenceCluster) {
                referenceElements.insert(elementId);
            }

            const auto sharedRefALines = clusterIntersectionByType(
                sourceGraph, referenceCluster, movingClusterA, false);
            const auto sharedRefBLines = clusterIntersectionByType(
                sourceGraph, referenceCluster, movingClusterB, false);
            const auto sharedABLines = clusterIntersectionByType(
                sourceGraph, movingClusterA, movingClusterB, false);

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

    [[nodiscard]] bool writeClusterPoseToGraph(
        const ClusterPose& pose, ConstraintGraph& targetGraph)
    {
        for (const auto& [elementId, elementPose] : pose) {
            const auto element = targetGraph.getElement(elementId);
            if (!element) {
                return false;
            }

            if (const auto point = poseAsPoint(elementPose);
                point.has_value()) {
                element->updateElementPosition(point->position);
                continue;
            }

            const auto line = poseAsLine(elementPose);
            if (!line.has_value()) {
                return false;
            }

            element->updateElementPosition(line->p1, line->p2);
        }

        return true;
    }

} // namespace

Constrainedness BottomUpDrPlanStrategy::checkConstraintGraphConstrainedness(
    const ConstraintGraph& gcs)
{
    const auto& deficit = (2 * gcs.nodeCount() - 3) - gcs.edgeCount();
    Constrainedness out {};
    if (deficit < 0) {
        out = Constrainedness::INCONSISTENTLY_OVER_CONSTRAINED;
    } else if (deficit == 0) {
        out = Constrainedness::WELL_CONSTRAINED;
    } else if (deficit > 0) {
        out = Constrainedness::UNDER_CONSTRAINED;
    }

    return out;
}

bool BottomUpDrPlanStrategy::resolve(ConstraintGraph& /*gcs*/)
{
    return false;
}

std::vector<ConstraintGraph> BottomUpDrPlanStrategy::decomposeConstraintGraph(
    ConstraintGraph& gcs)
{
    m_lastDecomposedGraph = &gcs;
    m_lastReductionResult = reduceBottomUp(gcs);

    return {};
}

void BottomUpDrPlanStrategy::solveGcs(
    std::vector<ConstraintGraph>& /*splitComponents*/)
{
    if (m_lastDecomposedGraph == nullptr
        || !m_lastReductionResult.has_value()) {
        throw std::runtime_error(
            "Bottom-up solve called without prior decomposition");
    }

    for (const auto& rootPlan : m_lastReductionResult->rootPlans) {
        const auto rootNode = rootPlan.getRoot();
        if (!rootNode.has_value()) {
            continue;
        }

        std::unordered_map<MathUtils::GeneralTreeNodeId, ClusterPose>
            solvedNodePose;

        const auto postOrderNodes = rootPlan.traversePostOrder();
        for (const auto& nodeId : postOrderNodes) {
            const auto& node = rootPlan.getValue(nodeId);

            if (node.kind == PlanNodeKind::EdgePrimitive) {
                const auto edgeInfo = std::get<EdgePrimitiveInfo>(node.info);
                const auto solved = solveEdgePrimitive(
                    *m_lastDecomposedGraph, edgeInfo.elements);
                if (!solved.has_value()) {
                    throw std::runtime_error(std::format(
                        "Failed to solve edge primitive cluster C{}",
                        edgeInfo.cluster.value));
                }

                solvedNodePose.emplace(nodeId, solved.value());
                continue;
            }

            if (node.kind == PlanNodeKind::TrianglePrimitive) {
                const auto triangleInfo
                    = std::get<TrianglePrimitiveInfo>(node.info);
                const auto solved = solveTrianglePrimitive(
                    *m_lastDecomposedGraph, triangleInfo.elements);
                if (!solved.has_value()) {
                    throw std::runtime_error(std::format(
                        "Failed to solve triangle primitive cluster C{}",
                        triangleInfo.cluster.value));
                }

                solvedNodePose.emplace(nodeId, solved.value());
                continue;
            }

            if (node.kind == PlanNodeKind::Merge3) {
                const auto children = rootPlan.children(nodeId);
                if (children.size() != 3) {
                    throw std::runtime_error(
                        "Merge3 node does not contain exactly three children");
                }

                if (!solvedNodePose.contains(children[0])
                    || !solvedNodePose.contains(children[1])
                    || !solvedNodePose.contains(children[2])) {
                    throw std::runtime_error(
                        "Merge3 node missing solved child clusters");
                }

                const auto pppMergeSolution = trySolveMerge3Ppp(
                    *m_lastDecomposedGraph, node, children, solvedNodePose);
                if (pppMergeSolution.has_value()) {
                    solvedNodePose.emplace(nodeId, pppMergeSolution.value());
                    continue;
                }

                const auto pllMergeSolution = trySolveMerge3Pll(
                    *m_lastDecomposedGraph, node, children, solvedNodePose);
                if (pllMergeSolution.has_value()) {
                    solvedNodePose.emplace(nodeId, pllMergeSolution.value());
                    continue;
                }

                const auto lppMergeSolution = trySolveMerge3Lpp(
                    *m_lastDecomposedGraph, node, children, solvedNodePose);
                if (lppMergeSolution.has_value()) {
                    solvedNodePose.emplace(nodeId, lppMergeSolution.value());
                    continue;
                }

                const auto llpMergeSolution = trySolveMerge3Llp(
                    *m_lastDecomposedGraph, node, children, solvedNodePose);
                if (llpMergeSolution.has_value()) {
                    solvedNodePose.emplace(nodeId, llpMergeSolution.value());
                    continue;
                }

                if (detectUnsolvableMerge3Lll(*m_lastDecomposedGraph, node,
                        children, solvedNodePose)) {
                    const auto mergeInfo = std::get<Merge3Info>(node.info);
                    throw std::runtime_error(std::format(
                        "Unsupported Merge3 LLL scenario in cluster C{}",
                        mergeInfo.output.value));
                }

                auto mergedCluster = solvedNodePose.at(children[0]);
                const auto firstMerge = mergeChildClusterIntoReference(
                    std::move(mergedCluster), solvedNodePose.at(children[1]));
                if (!firstMerge.has_value()) {
                    throw std::runtime_error(
                        "Failed to align first child in Merge3 node");
                }

                const auto secondMerge = mergeChildClusterIntoReference(
                    firstMerge.value(), solvedNodePose.at(children[2]));
                if (!secondMerge.has_value()) {
                    throw std::runtime_error(
                        "Failed to align second child in Merge3 node");
                }

                solvedNodePose.emplace(nodeId, secondMerge.value());
                continue;
            }

            throw std::runtime_error("Unsupported bottom-up plan node kind");
        }

        if (!solvedNodePose.contains(rootNode.value())) {
            throw std::runtime_error("Bottom-up root node was not solved");
        }

        if (!writeClusterPoseToGraph(
                solvedNodePose.at(rootNode.value()), *m_lastDecomposedGraph)) {
            throw std::runtime_error(
                "Failed to write merged bottom-up solution to graph");
        }
    }
}

} // namespace Gcs
