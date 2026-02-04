#include "point_point_solvers.hpp"
#include "../equations/equation_primitives.hpp"
#include "../equations/newton_raphson.hpp"
#include "constraints.hpp"
#include "elements.hpp"
#include "gcs_data_structures.hpp"
#include "heuristics.hpp"
#include "solve_result.hpp"
#include <algorithm>
#include <iostream>
#include <ranges>

namespace Gcs::Solvers {
bool ZeroFixedPointsTriangleSolver::matches(const ConstraintGraph& component)
{
    // Note that we assume that there are no double edges here
    return component.nodeCount() == 3 && component.edgeCount() == 3
        && component.numberOfSolvedElements() == 0
        && std::ranges::all_of(component.getElements(),
            [](const auto& e) { return e->template isElementType<Point>(); })
        && std::ranges::all_of(component.getConstraints(), [](const auto& c) {
               return c->template isConstraintType<DistanceConstraint>();
           });
}

SolveResult ZeroFixedPointsTriangleSolver::solve(ConstraintGraph& component)
{
    const auto& nodesToElementsMap = component.getElementMap();
    auto nodes = nodesToElementsMap | std::views::keys;

    int p1Idx = 0;
    int p2Idx = 1;
    int p3Idx = 2;

    // We assume that the matches function was called and the predicates hold
    auto point1 = component.getElement(nodes[p1Idx]);
    auto point2 = component.getElement(nodes[p2Idx]);
    auto point3 = component.getElement(nodes[p3Idx]);

    auto p1P2Distance
        = component.getConstraintBetweenNodes(nodes[p1Idx], nodes[p2Idx]);
    auto p1P3Distance
        = component.getConstraintBetweenNodes(nodes[p1Idx], nodes[p3Idx]);
    auto p2P3Distance
        = component.getConstraintBetweenNodes(nodes[p2Idx], nodes[p3Idx]);

    // Fix P1 at origin and P2 on the x-axis at distance d(P1,P2)
    point1->updateElementPosition(Eigen::Vector2d { 0.0, 0.0 });
    point2->updateElementPosition(
        Eigen::Vector2d { p1P2Distance->getConstraintValue().value(), 0.0 });

    const auto& point1Data = point1->getElement<Point>();
    const auto& point2Data = point2->getElement<Point>();
    const auto& point3Data = point3->getElement<Point>();

    auto p1P3DistanceFunction = Equations::pointToPointDistance(
        point1Data.position.x(), point1Data.position.y(),
        p1P3Distance->getConstraintValue().value());

    auto p2P3DistanceFunction = Equations::pointToPointDistance(
        point2Data.position.x(), point2Data.position.y(),
        p2P3Distance->getConstraintValue().value());

    auto possibleResults
        = Equations::solve2D(p1P3DistanceFunction, p2P3DistanceFunction);

    // Pick the candidate that preserves the original canvas orientation
    auto correctResult = pickByTriangleOrientation(point1Data.canvasPosition,
        point2Data.canvasPosition, point3Data.canvasPosition,
        point1Data.position, point2Data.position, possibleResults[0],
        possibleResults[1]);

    point3->updateElementPosition(
        Eigen::Vector2d { correctResult.x(), correctResult.y() });

    std::cerr << std::format(
        "Correct result is ({}, {})", correctResult.x(), correctResult.y());
    std::cerr << std::format(
        "Triangle Point 1: ({},{}), Point 2: ({},{}), Point 2: ({},{})",
        point1Data.position.x(), point1Data.position.y(),
        point2Data.position.x(), point2Data.position.y(),
        point3Data.position.x(), point3Data.position.y());

    return SolveResult::success();
}

bool TwoFixedPointsDistanceSolver::matches(const ConstraintGraph& component)
{
    return component.nodeCount() == 3 && component.numberOfSolvedElements() >= 2
        && std::ranges::all_of(component.getElements(),
            [](const auto& e) { return e->template isElementType<Point>(); })
        && std::ranges::all_of(component.getConstraints(), [](const auto& c) {
               return c->template isConstraintType<DistanceConstraint>();
           });
}

SolveResult TwoFixedPointsDistanceSolver::solve(ConstraintGraph& component)
{
    const auto& nodesToElementsMap = component.getElementMap();

    // Identify which points are already solved (fixed) and which is free
    std::shared_ptr<Element> fixedPoint1;
    std::shared_ptr<Element> fixedPoint2;
    std::shared_ptr<Element> freePoint;

    ConstraintGraph::NodeIdType fixedNode1 {};
    ConstraintGraph::NodeIdType fixedNode2 {};
    ConstraintGraph::NodeIdType freeNode {};

    for (const auto& [node, element] : nodesToElementsMap) {
        if (element->isElementSet()) {
            if (!fixedPoint1) {
                fixedPoint1 = element;
                fixedNode1 = node;
            } else {
                fixedPoint2 = element;
                fixedNode2 = node;
            }
        } else {
            freePoint = element;
            freeNode = node;
        }
    }

    // We assume that the matches function was called and the predicates hold
    auto fixedPoint1ToFreeDistance
        = component.getConstraintBetweenNodes(fixedNode1, freeNode);
    auto fixedPoint2ToFreeDistance
        = component.getConstraintBetweenNodes(fixedNode2, freeNode);

    const auto& fixedPoint1Data = fixedPoint1->getElement<Point>();
    const auto& fixedPoint2Data = fixedPoint2->getElement<Point>();
    const auto& freePointData = freePoint->getElement<Point>();

    // Build distance equations from each fixed point to the free point
    auto fixedPoint1ToFreeDistanceFunction = Equations::pointToPointDistance(
        fixedPoint1Data.position.x(), fixedPoint1Data.position.y(),
        fixedPoint1ToFreeDistance->getConstraintValue().value());

    auto fixedPoint2ToFreeDistanceFunction = Equations::pointToPointDistance(
        fixedPoint2Data.position.x(), fixedPoint2Data.position.y(),
        fixedPoint2ToFreeDistance->getConstraintValue().value());

    auto possibleResults = Equations::solve2D(
        fixedPoint1ToFreeDistanceFunction, fixedPoint2ToFreeDistanceFunction);

    // Pick the candidate that preserves the original canvas orientation
    auto correctResult = pickByTriangleOrientation(
        fixedPoint1Data.canvasPosition, fixedPoint2Data.canvasPosition,
        freePointData.canvasPosition, fixedPoint1Data.position,
        fixedPoint2Data.position, possibleResults[0], possibleResults[1]);

    freePoint->updateElementPosition(
        Eigen::Vector2d { correctResult.x(), correctResult.y() });

    std::cerr << std::format(
        "Correct result is ({}, {})", correctResult.x(), correctResult.y());
    std::cerr << std::format(
        "Triangle Point 1: ({},{}), Point 2: ({},{}), Point 2: ({},{})",
        fixedPoint1Data.position.x(), fixedPoint1Data.position.y(),
        fixedPoint2Data.position.x(), fixedPoint2Data.position.y(),
        freePointData.position.x(), freePointData.position.y());
    return SolveResult::success();
}

} // namespace Gcs::Solvers
