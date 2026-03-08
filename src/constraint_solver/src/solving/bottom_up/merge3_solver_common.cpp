#include "solving/bottom_up/merge3_solver_common.hpp"

// General STD/STL headers
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

// Custom headers
#include "solving/equations/equation_primitives.hpp"
#include "solving/equations/newton_raphson.hpp"
#include "solving/solvers/heuristics.hpp"

// Thirdparty headers
#include <Eigen/SVD>

namespace Gcs::Solvers::BottomUp {

namespace {

    [[nodiscard]] double signForDistance(double value)
    {
        return (value > 0.0) ? 1.0 : -1.0;
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

} // namespace

std::optional<LinePose> poseAsLine(const ElementPose& pose)
{
    if (!std::holds_alternative<LinePose>(pose)) {
        return std::nullopt;
    }

    return std::get<LinePose>(pose);
}

std::optional<PointPose> poseAsPoint(const ElementPose& pose)
{
    if (!std::holds_alternative<PointPose>(pose)) {
        return std::nullopt;
    }

    return std::get<PointPose>(pose);
}

Eigen::Vector2d lineMidpoint(const LinePose& line)
{
    return (line.p1 + line.p2) / 2.0;
}

std::optional<Eigen::Vector2d> lineUnitDirection(const LinePose& line)
{
    const Eigen::Vector2d direction = line.p2 - line.p1;
    const double length = direction.norm();
    if (length < EPSILON) {
        return std::nullopt;
    }

    return direction / length;
}

std::optional<RigidTransform> estimateRigidTransform(
    const std::vector<Eigen::Vector2d>& sourcePoints,
    const std::vector<Eigen::Vector2d>& targetPoints)
{
    if (sourcePoints.size() != targetPoints.size() || sourcePoints.empty()) {
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

ElementPose applyRigidTransform(
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

std::optional<ClusterPose> mergeChildClusterIntoReference(
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
        if (!movingDirection.has_value() || !referenceDirection.has_value()) {
            return std::nullopt;
        }

        const Eigen::Vector2d movingCenter = lineMidpoint(movingLine.value());
        const Eigen::Vector2d referenceCenter
            = lineMidpoint(referenceLine.value());

        sourcePoints.push_back(movingCenter);
        targetPoints.push_back(referenceCenter);
        sourcePoints.push_back(movingCenter + movingDirection.value());
        targetPoints.push_back(referenceCenter + referenceDirection.value());
    }

    const auto transform = estimateRigidTransform(sourcePoints, targetPoints);
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

std::optional<Eigen::Vector2d> getPointPosition(
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

std::optional<Eigen::Vector2d> getPointCanvasPosition(
    const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId)
{
    const auto element = graph.getElement(elementId);
    if (!element || !element->isElementType<Point>()) {
        return std::nullopt;
    }

    return element->getElement<Point>().canvasPosition;
}

std::optional<LinePose> getLinePosition(
    const ClusterPose& cluster, ConstraintGraph::NodeIdType elementId)
{
    if (!cluster.contains(elementId)) {
        return std::nullopt;
    }

    return poseAsLine(cluster.at(elementId));
}

std::optional<LinePose> getLineCanvasPose(
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

bool isPointElement(
    const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId)
{
    const auto element = graph.getElement(elementId);
    return element && element->isElementType<Point>();
}

bool isLineElement(
    const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId)
{
    const auto element = graph.getElement(elementId);
    return element && element->isElementType<Line>();
}

std::vector<ConstraintGraph::NodeIdType> clusterIntersectionByType(
    const ConstraintGraph& graph, const ClusterPose& first,
    const ClusterPose& second, bool selectPoints)
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

std::optional<ClusterPose> transformClusterByTwoPointAnchors(
    const ClusterPose& movingCluster, ConstraintGraph::NodeIdType fixedPoint,
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

    const auto transform = estimateRigidTransform(sourcePoints, targetPoints);
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

std::optional<ClusterPose> transformClusterByAnchors(
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

        const Eigen::Vector2d sourceCenter = lineMidpoint(sourceLine.value());
        const Eigen::Vector2d targetCenter = lineMidpoint(targetLine.value());
        sourcePoints.push_back(sourceCenter);
        targetPoints.push_back(targetCenter);
        sourcePoints.push_back(sourceCenter + sourceDirection.value());
        targetPoints.push_back(targetCenter + targetDirection.value());
    }

    const auto transform = estimateRigidTransform(sourcePoints, targetPoints);
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

double scoreMergedPose(
    const ConstraintGraph& sourceGraph, const ClusterPose& mergedPose)
{
    double score = 0.0;
    std::size_t scoreTerms = 0;

    for (const auto& [elementId, pose] : mergedPose) {
        if (const auto pointPose = poseAsPoint(pose); pointPose.has_value()) {
            const auto canvasPoint
                = getPointCanvasPosition(sourceGraph, elementId);
            if (!canvasPoint.has_value()) {
                continue;
            }

            score += (pointPose->position - canvasPoint.value()).squaredNorm();
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

double safeCanvasLineLength(const Line& line)
{
    const double canvasLength = (line.canvasP2 - line.canvasP1).norm();
    if (canvasLength < EPSILON) {
        return MIN_LINE_LENGTH;
    }

    return canvasLength;
}

double lineLength(const LinePose& line)
{
    const double length = (line.p2 - line.p1).norm();
    return (length < EPSILON) ? MIN_LINE_LENGTH : length;
}

double pointToLineDistanceAbs(
    const Eigen::Vector2d& point, const LinePose& line)
{
    return std::abs(Solvers::signedDistanceToLine(point, line.p1, line.p2));
}

std::optional<LinePose> solveFreeLineFromFixedPoints(
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

    const double offset0 = candidateNormals[0].dot(fixedPointA) - signedDistA;
    const double offset1 = candidateNormals[1].dot(fixedPointA) - signedDistA;

    const auto [normalX, normalY, offset] = Solvers::pickLineBySignedDistances(
        canvasSignedA, canvasSignedB, candidateNormals[0], candidateNormals[1],
        fixedPointA, fixedPointB, offset0, offset1);

    const auto [lineP1, lineP2] = reconstructLineEndpoints(fixedPointA,
        fixedPointB, normalX, normalY, offset, lineLength(canvasFreeLine));

    return LinePose {
        .p1 = lineP1,
        .p2 = lineP2,
    };
}

std::optional<Eigen::Vector2d> solveFreePointFromFixedPointAndLine(
    const Eigen::Vector2d& fixedPoint, const LinePose& fixedLine,
    double distanceToPoint, double distanceToLine,
    const Eigen::Vector2d& canvasFixedPoint, const LinePose& canvasFixedLine,
    const Eigen::Vector2d& canvasFreePoint)
{
    const double canvasSigned = Solvers::signedDistanceToLine(
        canvasFreePoint, canvasFixedLine.p1, canvasFixedLine.p2);
    const double signedDistance
        = signForDistance(canvasSigned) * distanceToLine;

    auto pointEquation = Equations::pointToPointDistance(
        fixedPoint.x(), fixedPoint.y(), distanceToPoint);
    auto lineEquation = Equations::pointToLineDistance(fixedLine.p1.x(),
        fixedLine.p1.y(), fixedLine.p2.x(), fixedLine.p2.y(), signedDistance,
        lineLength(fixedLine));

    const auto candidates = Equations::solve2D(pointEquation, lineEquation);

    const Eigen::Vector2d solverFoot
        = Solvers::perpendicularFoot(fixedPoint, fixedLine.p1, fixedLine.p2);
    const Eigen::Vector2d canvasFoot = Solvers::perpendicularFoot(
        canvasFixedPoint, canvasFixedLine.p1, canvasFixedLine.p2);

    return Solvers::pickByTriangleOrientationWithFallback(canvasFixedPoint,
        canvasFoot, canvasFreePoint, fixedPoint, solverFoot, candidates[0],
        candidates[1]);
}

std::optional<Eigen::Vector2d> solveFreePointFromFixedLines(
    const LinePose& fixedLineA, const LinePose& fixedLineB,
    double distanceToLineA, double distanceToLineB, const LinePose& canvasLineA,
    const LinePose& canvasLineB, const Eigen::Vector2d& canvasFreePoint)
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

} // namespace Gcs::Solvers::BottomUp
