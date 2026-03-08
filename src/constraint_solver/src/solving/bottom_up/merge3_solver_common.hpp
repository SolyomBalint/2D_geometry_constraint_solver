#ifndef MERGE3_SOLVER_COMMON_HPP
#define MERGE3_SOLVER_COMMON_HPP

// General STD/STL headers
#include <optional>
#include <span>
#include <utility>
#include <vector>

// Thirdparty headers
#include <Eigen/Core>

// Custom headers
#include "model/elements.hpp"
#include "solving/bottom_up/plan_pose_types.hpp"

namespace Gcs::Solvers::BottomUp {

struct RigidTransform {
    Eigen::Matrix2d rotation;
    Eigen::Vector2d translation;
};

constexpr double MIN_LINE_LENGTH = 50.0;
constexpr double EPSILON = 1e-9;

[[nodiscard]] std::optional<LinePose> poseAsLine(const ElementPose& pose);
[[nodiscard]] std::optional<PointPose> poseAsPoint(const ElementPose& pose);
[[nodiscard]] Eigen::Vector2d lineMidpoint(const LinePose& line);
[[nodiscard]] std::optional<Eigen::Vector2d> lineUnitDirection(
    const LinePose& line);

[[nodiscard]] std::optional<RigidTransform> estimateRigidTransform(
    const std::vector<Eigen::Vector2d>& sourcePoints,
    const std::vector<Eigen::Vector2d>& targetPoints);
[[nodiscard]] ElementPose applyRigidTransform(
    const ElementPose& pose, const RigidTransform& transform);

[[nodiscard]] std::optional<ClusterPose> mergeChildClusterIntoReference(
    ClusterPose referenceCluster, const ClusterPose& movingCluster);

[[nodiscard]] std::optional<Eigen::Vector2d> getPointPosition(
    const ClusterPose& cluster, ConstraintGraph::NodeIdType elementId);
[[nodiscard]] std::optional<Eigen::Vector2d> getPointCanvasPosition(
    const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId);
[[nodiscard]] std::optional<LinePose> getLinePosition(
    const ClusterPose& cluster, ConstraintGraph::NodeIdType elementId);
[[nodiscard]] std::optional<LinePose> getLineCanvasPose(
    const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId);

[[nodiscard]] bool isPointElement(
    const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId);
[[nodiscard]] bool isLineElement(
    const ConstraintGraph& graph, ConstraintGraph::NodeIdType elementId);

[[nodiscard]] std::vector<ConstraintGraph::NodeIdType>
clusterIntersectionByType(const ConstraintGraph& graph,
    const ClusterPose& first, const ClusterPose& second, bool selectPoints);

[[nodiscard]] std::optional<ClusterPose> transformClusterByTwoPointAnchors(
    const ClusterPose& movingCluster, ConstraintGraph::NodeIdType fixedPoint,
    ConstraintGraph::NodeIdType freePoint,
    const Eigen::Vector2d& fixedPointGlobal,
    const Eigen::Vector2d& freePointGlobal);

[[nodiscard]] std::optional<ClusterPose> transformClusterByAnchors(
    const ClusterPose& movingCluster,
    std::span<const std::pair<ConstraintGraph::NodeIdType, ElementPose>>
        anchors);

[[nodiscard]] double scoreMergedPose(
    const ConstraintGraph& sourceGraph, const ClusterPose& mergedPose);

[[nodiscard]] double safeCanvasLineLength(const Line& line);
[[nodiscard]] double lineLength(const LinePose& line);
[[nodiscard]] double pointToLineDistanceAbs(
    const Eigen::Vector2d& point, const LinePose& line);

[[nodiscard]] std::optional<LinePose> solveFreeLineFromFixedPoints(
    const Eigen::Vector2d& fixedPointA, const Eigen::Vector2d& fixedPointB,
    double distanceA, double distanceB, const Eigen::Vector2d& canvasPointA,
    const Eigen::Vector2d& canvasPointB, const LinePose& canvasFreeLine);

[[nodiscard]] std::optional<Eigen::Vector2d>
solveFreePointFromFixedPointAndLine(const Eigen::Vector2d& fixedPoint,
    const LinePose& fixedLine, double distanceToPoint, double distanceToLine,
    const Eigen::Vector2d& canvasFixedPoint, const LinePose& canvasFixedLine,
    const Eigen::Vector2d& canvasFreePoint);

[[nodiscard]] std::optional<Eigen::Vector2d> solveFreePointFromFixedLines(
    const LinePose& fixedLineA, const LinePose& fixedLineB,
    double distanceToLineA, double distanceToLineB, const LinePose& canvasLineA,
    const LinePose& canvasLineB, const Eigen::Vector2d& canvasFreePoint);

} // namespace Gcs::Solvers::BottomUp

#endif // MERGE3_SOLVER_COMMON_HPP
