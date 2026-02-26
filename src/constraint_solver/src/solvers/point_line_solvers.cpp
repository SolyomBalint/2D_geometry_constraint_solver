#include "point_line_solvers.hpp"
#include "../equations/equation_primitives.hpp"
#include "../equations/newton_raphson.hpp"
#include "constraints.hpp"
#include "elements.hpp"
#include "gcs_data_structures.hpp"
#include "heuristics.hpp"
#include "solve_result.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <format>
#include <iostream>
#include <ranges>

namespace Gcs::Solvers {

// ================================================================
// Helper: count element types in a component
// ================================================================

namespace {

    struct ElementTypeCounts {
        int pointCount = 0;
        int lineCount = 0;
    };

    ElementTypeCounts countElementTypes(const ConstraintGraph& component)
    {
        ElementTypeCounts counts;
        for (const auto& element : component.getElements()) {
            if (element->template isElementType<Point>()) {
                ++counts.pointCount;
            } else if (element->template isElementType<Line>()) {
                ++counts.lineCount;
            }
        }
        return counts;
    }

    bool allNonVirtualConstraintsAreDistance(const ConstraintGraph& component)
    {
        const auto& constraintMap = component.getConstraintMap();
        for (const auto& [edgeId, constraint] : constraintMap) {
            if (component.isVirtualEdge(edgeId)) {
                continue;
            }
            if (!constraint->template isConstraintType<DistanceConstraint>()) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Reconstruct line segment endpoints in solver space.
     *
     * Given the solved infinite line (by its unit normal and
     * perpendicular offset) and the two constraining points (in
     * solver space), projects those points onto the line and
     * extends the segment to match the original canvas line length.
     *
     * @param constrainingPoint1 First solver-space point that
     *        constrains this line (e.g., a fixed point).
     * @param constrainingPoint2 Second solver-space point.
     * @param normalX X component of the solved unit normal.
     * @param normalY Y component of the solved unit normal.
     * @param perpendicularOffset The line's perpendicular offset.
     * @param canvasLineLength Length of the original canvas line.
     * @return Pair of (solverP1, solverP2) line endpoints.
     */
    std::pair<Eigen::Vector2d, Eigen::Vector2d> reconstructLineEndpoints(
        const Eigen::Vector2d& constrainingPoint1,
        const Eigen::Vector2d& constrainingPoint2, double normalX,
        double normalY, double perpendicularOffset, double canvasLineLength)
    {
        Eigen::Vector2d normal { normalX, normalY };

        // Project both constraining points onto the solved line
        auto projectOntoLine
            = [&normal, perpendicularOffset](const Eigen::Vector2d& point) {
                  double signedDist = normal.dot(point) - perpendicularOffset;
                  return point - signedDist * normal;
              };

        Eigen::Vector2d projection1 = projectOntoLine(constrainingPoint1);
        Eigen::Vector2d projection2 = projectOntoLine(constrainingPoint2);

        // Line direction perpendicular to the normal
        Eigen::Vector2d lineDirection { -normalY, normalX };

        // Midpoint of the two projections along the line
        Eigen::Vector2d midpoint = (projection1 + projection2) / 2.0;

        // Use the larger of the canvas line length or the span
        // between projections, so the line always covers the
        // relevant region
        double projectionSpan
            = std::abs(lineDirection.dot(projection2 - projection1));
        double halfLength = std::max(canvasLineLength, projectionSpan) / 2.0;

        return { midpoint - halfLength * lineDirection,
            midpoint + halfLength * lineDirection };
    }

} // anonymous namespace

// ================================================================
// ZeroFixedPPLTriangleSolver
// ================================================================

bool ZeroFixedPPLTriangleSolver::matches(const ConstraintGraph& component)
{
    if (component.nodeCount() != 3 || component.edgeCount() != 3) {
        return false;
    }

    if (component.numberOfSolvedElements() != 0) {
        return false;
    }

    auto [pointCount, lineCount] = countElementTypes(component);
    if (pointCount != 2 || lineCount != 1) {
        return false;
    }

    return std::ranges::all_of(
        component.getConstraints(), [](const auto& constraint) {
            return constraint->template isConstraintType<DistanceConstraint>();
        });
}

SolveResult ZeroFixedPPLTriangleSolver::solve(ConstraintGraph& component)
{
    const auto& nodesToElementsMap = component.getElementMap();

    // Separate the two points and the line
    std::shared_ptr<Element> point1Element;
    std::shared_ptr<Element> point2Element;
    std::shared_ptr<Element> lineElement;

    ConstraintGraph::NodeIdType point1Node {};
    ConstraintGraph::NodeIdType point2Node {};
    ConstraintGraph::NodeIdType lineNode {};

    for (const auto& [node, element] : nodesToElementsMap) {
        if (element->template isElementType<Line>()) {
            lineElement = element;
            lineNode = node;
        } else if (element->template isElementType<Point>()) {
            if (!point1Element) {
                point1Element = element;
                point1Node = node;
            } else {
                point2Element = element;
                point2Node = node;
            }
        }
    }

    // Get the three distance constraints
    auto point1ToPoint2Constraint
        = component.getConstraintBetweenNodes(point1Node, point2Node);
    auto point1ToLineConstraint
        = component.getConstraintBetweenNodes(point1Node, lineNode);
    auto point2ToLineConstraint
        = component.getConstraintBetweenNodes(point2Node, lineNode);

    double point1ToPoint2Distance
        = point1ToPoint2Constraint->getConstraintValue().value();
    double point1ToLineDistance
        = point1ToLineConstraint->getConstraintValue().value();
    double point2ToLineDistance
        = point2ToLineConstraint->getConstraintValue().value();

    // Fix point 1 at the origin and point 2 on the X-axis
    point1Element->updateElementPosition(Eigen::Vector2d { 0.0, 0.0 });
    point2Element->updateElementPosition(
        Eigen::Vector2d { point1ToPoint2Distance, 0.0 });

    const auto& point1Data = point1Element->getElement<Point>();
    const auto& point2Data = point2Element->getElement<Point>();
    const auto& lineData = lineElement->getElement<Line>();

    // Determine signed distances from the canvas layout to choose
    // which side of the line each point falls on
    double canvasSignedDistFromPoint1 = signedDistanceToLine(
        point1Data.canvasPosition, lineData.canvasP1, lineData.canvasP2);
    double canvasSignedDistFromPoint2 = signedDistanceToLine(
        point2Data.canvasPosition, lineData.canvasP1, lineData.canvasP2);

    // Build the signed distances preserving the canvas sign pattern
    auto signOf = [](double x) -> double { return (x > 0.0) ? 1.0 : -1.0; };
    double signedDistPoint1ToLine
        = signOf(canvasSignedDistFromPoint1) * point1ToLineDistance;
    double signedDistPoint2ToLine
        = signOf(canvasSignedDistFromPoint2) * point2ToLineDistance;

    // Solve for the line's unit normal (nx, ny).
    // Equation 1: nx * (P2x - P1x) + ny * (P2y - P1y)
    //             + signedDist1 - signedDist2 = 0
    // Equation 2: nx^2 + ny^2 - 1 = 0
    Eigen::Vector2d delta = point2Data.position - point1Data.position;

    auto normalDistanceEquation = Equations::lineNormalSignedDistanceDiff(
        delta.x(), delta.y(), signedDistPoint1ToLine, signedDistPoint2ToLine);

    auto unitConstraintEquation = Equations::unitNormalConstraint();

    // Use the canvas line's unit normal as initial guesses
    Eigen::Vector2d canvasLineDirection = lineData.canvasP2 - lineData.canvasP1;
    double canvasLineLength = canvasLineDirection.norm();
    Eigen::Vector2d canvasNormal { -canvasLineDirection.y() / canvasLineLength,
        canvasLineDirection.x() / canvasLineLength };

    std::array<Eigen::Vector2d, 2> normalInitialGuesses { canvasNormal,
        -canvasNormal };

    auto candidateNormals = Equations::solve2D(
        normalDistanceEquation, unitConstraintEquation, normalInitialGuesses);

    // Compute perpendicular offsets for each candidate:
    // p = n . P1 - signedDist1 (signed distance from origin)
    double perpendicularOffset0
        = candidateNormals[0].dot(point1Data.position) - signedDistPoint1ToLine;
    double perpendicularOffset1
        = candidateNormals[1].dot(point1Data.position) - signedDistPoint1ToLine;

    // Disambiguate: pick the candidate whose sign pattern matches
    // the canvas layout
    auto [chosenNormalX, chosenNormalY, chosenOffset]
        = pickLineBySignedDistances(canvasSignedDistFromPoint1,
            canvasSignedDistFromPoint2, candidateNormals[0],
            candidateNormals[1], point1Data.position, point2Data.position,
            perpendicularOffset0, perpendicularOffset1);

    // Reconstruct line endpoints in solver space by projecting the
    // constraining solver-space points onto the solved line and
    // extending to match the canvas line length.
    auto [solverLineP1, solverLineP2]
        = reconstructLineEndpoints(point1Data.position, point2Data.position,
            chosenNormalX, chosenNormalY, chosenOffset, canvasLineLength);

    lineElement->updateElementPosition(solverLineP1, solverLineP2);

    std::cerr << std::format("PPL Triangle: P1=({},{}), P2=({},{}), "
                             "Line=({},{})--({},{})\n",
        point1Data.position.x(), point1Data.position.y(),
        point2Data.position.x(), point2Data.position.y(), solverLineP1.x(),
        solverLineP1.y(), solverLineP2.x(), solverLineP2.y());

    return SolveResult::success();
}

// ================================================================
// TwoFixedPointsLineSolver
// ================================================================

bool TwoFixedPointsLineSolver::matches(const ConstraintGraph& component)
{
    if (component.nodeCount() != 3) {
        return false;
    }

    if (component.numberOfSolvedElements() < 2) {
        return false;
    }

    auto [pointCount, lineCount] = countElementTypes(component);
    if (pointCount != 2 || lineCount != 1) {
        return false;
    }

    // Both points must be solved, line must be unsolved
    for (const auto& element : component.getElements()) {
        if (element->template isElementType<Point>()
            && !element->isElementSet()) {
            return false;
        }
        if (element->template isElementType<Line>()
            && element->isElementSet()) {
            return false;
        }
    }

    return allNonVirtualConstraintsAreDistance(component);
}

SolveResult TwoFixedPointsLineSolver::solve(ConstraintGraph& component)
{
    const auto& nodesToElementsMap = component.getElementMap();

    // Separate the two fixed points and the free line
    std::shared_ptr<Element> fixedPoint1Element;
    std::shared_ptr<Element> fixedPoint2Element;
    std::shared_ptr<Element> freeLineElement;

    ConstraintGraph::NodeIdType fixedPoint1Node {};
    ConstraintGraph::NodeIdType fixedPoint2Node {};
    ConstraintGraph::NodeIdType freeLineNode {};

    for (const auto& [node, element] : nodesToElementsMap) {
        if (element->template isElementType<Line>()) {
            freeLineElement = element;
            freeLineNode = node;
        } else if (element->template isElementType<Point>()) {
            if (!fixedPoint1Element) {
                fixedPoint1Element = element;
                fixedPoint1Node = node;
            } else {
                fixedPoint2Element = element;
                fixedPoint2Node = node;
            }
        }
    }

    // Get the distance constraints from each point to the line
    auto fixedPoint1ToLineConstraint
        = component.getConstraintBetweenNodes(fixedPoint1Node, freeLineNode);
    auto fixedPoint2ToLineConstraint
        = component.getConstraintBetweenNodes(fixedPoint2Node, freeLineNode);

    double fixedPoint1ToLineDistance
        = fixedPoint1ToLineConstraint->getConstraintValue().value();
    double fixedPoint2ToLineDistance
        = fixedPoint2ToLineConstraint->getConstraintValue().value();

    const auto& fixedPoint1Data = fixedPoint1Element->getElement<Point>();
    const auto& fixedPoint2Data = fixedPoint2Element->getElement<Point>();
    const auto& freeLineData = freeLineElement->getElement<Line>();

    // Determine signed distances from the canvas layout
    double canvasSignedDistFromPoint1
        = signedDistanceToLine(fixedPoint1Data.canvasPosition,
            freeLineData.canvasP1, freeLineData.canvasP2);
    double canvasSignedDistFromPoint2
        = signedDistanceToLine(fixedPoint2Data.canvasPosition,
            freeLineData.canvasP1, freeLineData.canvasP2);

    auto signOf = [](double x) -> double { return (x > 0.0) ? 1.0 : -1.0; };
    double signedDistPoint1ToLine
        = signOf(canvasSignedDistFromPoint1) * fixedPoint1ToLineDistance;
    double signedDistPoint2ToLine
        = signOf(canvasSignedDistFromPoint2) * fixedPoint2ToLineDistance;

    // Solve for the line's unit normal (nx, ny)
    Eigen::Vector2d delta = fixedPoint2Data.position - fixedPoint1Data.position;

    auto normalDistanceEquation = Equations::lineNormalSignedDistanceDiff(
        delta.x(), delta.y(), signedDistPoint1ToLine, signedDistPoint2ToLine);

    auto unitConstraintEquation = Equations::unitNormalConstraint();

    // Use the canvas line's unit normal as initial guesses
    Eigen::Vector2d canvasLineDirection
        = freeLineData.canvasP2 - freeLineData.canvasP1;
    double canvasLineLength = canvasLineDirection.norm();
    Eigen::Vector2d canvasNormal { -canvasLineDirection.y() / canvasLineLength,
        canvasLineDirection.x() / canvasLineLength };

    std::array<Eigen::Vector2d, 2> normalInitialGuesses { canvasNormal,
        -canvasNormal };

    auto candidateNormals = Equations::solve2D(
        normalDistanceEquation, unitConstraintEquation, normalInitialGuesses);

    // Compute perpendicular offsets for each candidate
    double perpendicularOffset0
        = candidateNormals[0].dot(fixedPoint1Data.position)
        - signedDistPoint1ToLine;
    double perpendicularOffset1
        = candidateNormals[1].dot(fixedPoint1Data.position)
        - signedDistPoint1ToLine;

    // Disambiguate
    auto [chosenNormalX, chosenNormalY, chosenOffset]
        = pickLineBySignedDistances(canvasSignedDistFromPoint1,
            canvasSignedDistFromPoint2, candidateNormals[0],
            candidateNormals[1], fixedPoint1Data.position,
            fixedPoint2Data.position, perpendicularOffset0,
            perpendicularOffset1);

    // Reconstruct line endpoints in solver space
    auto [solverLineP1, solverLineP2] = reconstructLineEndpoints(
        fixedPoint1Data.position, fixedPoint2Data.position, chosenNormalX,
        chosenNormalY, chosenOffset, canvasLineLength);

    freeLineElement->updateElementPosition(solverLineP1, solverLineP2);

    std::cerr << std::format("TwoFixedPointsLine: P1=({},{}), P2=({},{}), "
                             "Line=({},{})--({},{})\n",
        fixedPoint1Data.position.x(), fixedPoint1Data.position.y(),
        fixedPoint2Data.position.x(), fixedPoint2Data.position.y(),
        solverLineP1.x(), solverLineP1.y(), solverLineP2.x(), solverLineP2.y());

    return SolveResult::success();
}

// ================================================================
// FixedPointAndLineFreePointSolver
// ================================================================

bool FixedPointAndLineFreePointSolver::matches(const ConstraintGraph& component)
{
    if (component.nodeCount() != 3) {
        return false;
    }

    if (component.numberOfSolvedElements() < 2) {
        return false;
    }

    auto [pointCount, lineCount] = countElementTypes(component);
    if (pointCount != 2 || lineCount != 1) {
        return false;
    }

    // Exactly one Point must be unsolved, one Point and the Line
    // must be solved
    int solvedPointCount = 0;
    int unsolvedPointCount = 0;
    bool lineIsSolved = false;

    for (const auto& element : component.getElements()) {
        if (element->template isElementType<Point>()) {
            if (element->isElementSet()) {
                ++solvedPointCount;
            } else {
                ++unsolvedPointCount;
            }
        } else if (element->template isElementType<Line>()) {
            lineIsSolved = element->isElementSet();
        }
    }

    if (solvedPointCount != 1 || unsolvedPointCount != 1 || !lineIsSolved) {
        return false;
    }

    return allNonVirtualConstraintsAreDistance(component);
}

SolveResult FixedPointAndLineFreePointSolver::solve(ConstraintGraph& component)
{
    const auto& nodesToElementsMap = component.getElementMap();

    // Identify the fixed point, fixed line, and free point
    std::shared_ptr<Element> fixedPointElement;
    std::shared_ptr<Element> freePointElement;
    std::shared_ptr<Element> fixedLineElement;

    ConstraintGraph::NodeIdType fixedPointNode {};
    ConstraintGraph::NodeIdType freePointNode {};
    ConstraintGraph::NodeIdType fixedLineNode {};

    for (const auto& [node, element] : nodesToElementsMap) {
        if (element->template isElementType<Line>()) {
            fixedLineElement = element;
            fixedLineNode = node;
        } else if (element->template isElementType<Point>()) {
            if (element->isElementSet()) {
                fixedPointElement = element;
                fixedPointNode = node;
            } else {
                freePointElement = element;
                freePointNode = node;
            }
        }
    }

    // Get constraints from the fixed elements to the free point
    auto fixedPointToFreePointConstraint
        = component.getConstraintBetweenNodes(fixedPointNode, freePointNode);
    auto fixedLineToFreePointConstraint
        = component.getConstraintBetweenNodes(fixedLineNode, freePointNode);

    double fixedPointToFreePointDistance
        = fixedPointToFreePointConstraint->getConstraintValue().value();
    double fixedLineToFreePointDistance
        = fixedLineToFreePointConstraint->getConstraintValue().value();

    const auto& fixedPointData = fixedPointElement->getElement<Point>();
    const auto& fixedLineData = fixedLineElement->getElement<Line>();
    const auto& freePointData = freePointElement->getElement<Point>();

    // Determine the signed distance from the canvas layout
    double canvasSignedDistFromFreePoint
        = signedDistanceToLine(freePointData.canvasPosition,
            fixedLineData.canvasP1, fixedLineData.canvasP2);

    auto signOf = [](double x) -> double { return (x > 0.0) ? 1.0 : -1.0; };
    double signedLineToFreePointDistance
        = signOf(canvasSignedDistFromFreePoint) * fixedLineToFreePointDistance;

    // Build equations for the free point's (x, y):
    // Equation 1: (x - fixedPt.x)^2 + (y - fixedPt.y)^2
    //             - distance^2 = 0
    auto pointToPointDistanceEquation
        = Equations::pointToPointDistance(fixedPointData.position.x(),
            fixedPointData.position.y(), fixedPointToFreePointDistance);

    // Equation 2: (p2x-p1x)*(y-p1y) - (p2y-p1y)*(x-p1x)
    //             - signedDist * lineLength = 0
    double fixedLineLength = fixedLineData.length();
    auto pointToLineDistanceEquation = Equations::pointToLineDistance(
        fixedLineData.p1.x(), fixedLineData.p1.y(), fixedLineData.p2.x(),
        fixedLineData.p2.y(), signedLineToFreePointDistance, fixedLineLength);

    auto candidatePositions = Equations::solve2D(
        pointToPointDistanceEquation, pointToLineDistanceEquation);

    // Disambiguate using triangle orientation with the line's
    // midpoint as the third vertex proxy
    Eigen::Vector2d fixedLineMidpoint = fixedLineData.midpoint();
    Eigen::Vector2d canvasLineMidpoint
        = (fixedLineData.canvasP1 + fixedLineData.canvasP2) / 2.0;

    auto correctResult = pickByTriangleOrientation(
        fixedPointData.canvasPosition, canvasLineMidpoint,
        freePointData.canvasPosition, fixedPointData.position,
        fixedLineMidpoint, candidatePositions[0], candidatePositions[1]);

    freePointElement->updateElementPosition(
        Eigen::Vector2d { correctResult.x(), correctResult.y() });

    std::cerr << std::format("FixedPointAndLine: fixedPt=({},{}), "
                             "line=({},{})--({},{}), freePt=({},{})\n",
        fixedPointData.position.x(), fixedPointData.position.y(),
        fixedLineData.p1.x(), fixedLineData.p1.y(), fixedLineData.p2.x(),
        fixedLineData.p2.y(), correctResult.x(), correctResult.y());

    return SolveResult::success();
}

// ================================================================
// TwoFixedLinesFreePointSolver
// ================================================================

bool TwoFixedLinesFreePointSolver::matches(const ConstraintGraph& component)
{
    if (component.nodeCount() != 3) {
        return false;
    }

    if (component.numberOfSolvedElements() < 2) {
        return false;
    }

    auto [pointCount, lineCount] = countElementTypes(component);
    if (pointCount != 1 || lineCount != 2) {
        return false;
    }

    // Point must be unsolved, both lines must be solved
    for (const auto& element : component.getElements()) {
        if (element->template isElementType<Point>()
            && element->isElementSet()) {
            return false;
        }
        if (element->template isElementType<Line>()
            && !element->isElementSet()) {
            return false;
        }
    }

    return allNonVirtualConstraintsAreDistance(component);
}

SolveResult TwoFixedLinesFreePointSolver::solve(ConstraintGraph& component)
{
    const auto& nodesToElementsMap = component.getElementMap();

    // Identify the two fixed lines and the free point
    std::shared_ptr<Element> fixedLine1Element;
    std::shared_ptr<Element> fixedLine2Element;
    std::shared_ptr<Element> freePointElement;

    ConstraintGraph::NodeIdType fixedLine1Node {};
    ConstraintGraph::NodeIdType fixedLine2Node {};
    ConstraintGraph::NodeIdType freePointNode {};

    for (const auto& [node, element] : nodesToElementsMap) {
        if (element->template isElementType<Point>()) {
            freePointElement = element;
            freePointNode = node;
        } else if (element->template isElementType<Line>()) {
            if (!fixedLine1Element) {
                fixedLine1Element = element;
                fixedLine1Node = node;
            } else {
                fixedLine2Element = element;
                fixedLine2Node = node;
            }
        }
    }

    // Get constraints from each line to the free point
    auto fixedLine1ToFreePointConstraint
        = component.getConstraintBetweenNodes(fixedLine1Node, freePointNode);
    auto fixedLine2ToFreePointConstraint
        = component.getConstraintBetweenNodes(fixedLine2Node, freePointNode);

    double fixedLine1ToFreePointDistance
        = fixedLine1ToFreePointConstraint->getConstraintValue().value();
    double fixedLine2ToFreePointDistance
        = fixedLine2ToFreePointConstraint->getConstraintValue().value();

    const auto& fixedLine1Data = fixedLine1Element->getElement<Line>();
    const auto& fixedLine2Data = fixedLine2Element->getElement<Line>();
    const auto& freePointData = freePointElement->getElement<Point>();

    // Determine signed distances from the canvas layout
    double canvasSignedDistFromLine1
        = signedDistanceToLine(freePointData.canvasPosition,
            fixedLine1Data.canvasP1, fixedLine1Data.canvasP2);
    double canvasSignedDistFromLine2
        = signedDistanceToLine(freePointData.canvasPosition,
            fixedLine2Data.canvasP1, fixedLine2Data.canvasP2);

    auto signOf = [](double x) -> double { return (x > 0.0) ? 1.0 : -1.0; };
    double signedLine1ToFreePointDistance
        = signOf(canvasSignedDistFromLine1) * fixedLine1ToFreePointDistance;
    double signedLine2ToFreePointDistance
        = signOf(canvasSignedDistFromLine2) * fixedLine2ToFreePointDistance;

    // Build equations for the free point's (x, y):
    // Both are point-to-line distance equations (linear in x, y)
    double fixedLine1Length = fixedLine1Data.length();
    auto line1DistanceEquation
        = Equations::pointToLineDistance(fixedLine1Data.p1.x(),
            fixedLine1Data.p1.y(), fixedLine1Data.p2.x(), fixedLine1Data.p2.y(),
            signedLine1ToFreePointDistance, fixedLine1Length);

    double fixedLine2Length = fixedLine2Data.length();
    auto line2DistanceEquation
        = Equations::pointToLineDistance(fixedLine2Data.p1.x(),
            fixedLine2Data.p1.y(), fixedLine2Data.p2.x(), fixedLine2Data.p2.y(),
            signedLine2ToFreePointDistance, fixedLine2Length);

    auto candidatePositions
        = Equations::solve2D(line1DistanceEquation, line2DistanceEquation);

    // Disambiguate using triangle orientation with line midpoints
    // as vertex proxies
    Eigen::Vector2d fixedLine1Midpoint = fixedLine1Data.midpoint();
    Eigen::Vector2d fixedLine2Midpoint = fixedLine2Data.midpoint();
    Eigen::Vector2d canvasLine1Midpoint
        = (fixedLine1Data.canvasP1 + fixedLine1Data.canvasP2) / 2.0;
    Eigen::Vector2d canvasLine2Midpoint
        = (fixedLine2Data.canvasP1 + fixedLine2Data.canvasP2) / 2.0;

    auto correctResult = pickByTriangleOrientation(canvasLine1Midpoint,
        canvasLine2Midpoint, freePointData.canvasPosition, fixedLine1Midpoint,
        fixedLine2Midpoint, candidatePositions[0], candidatePositions[1]);

    freePointElement->updateElementPosition(
        Eigen::Vector2d { correctResult.x(), correctResult.y() });

    std::cerr << std::format("TwoFixedLines: L1=({},{})--({},{}), "
                             "L2=({},{})--({},{}), freePt=({},{})\n",
        fixedLine1Data.p1.x(), fixedLine1Data.p1.y(), fixedLine1Data.p2.x(),
        fixedLine1Data.p2.y(), fixedLine2Data.p1.x(), fixedLine2Data.p1.y(),
        fixedLine2Data.p2.x(), fixedLine2Data.p2.y(), correctResult.x(),
        correctResult.y());

    return SolveResult::success();
}

} // namespace Gcs::Solvers
