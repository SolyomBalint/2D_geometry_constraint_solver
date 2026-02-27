#include "line_angle_solvers.hpp"
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
// Helpers (anonymous namespace)
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

    /**
     * @brief Check that non-virtual constraints are exactly
     *        1 AngleConstraint and 2 DistanceConstraints.
     *
     * Used by the anchor solver to validate the expected
     * constraint pattern: angle between the two lines, plus
     * distance from the point to each line.
     *
     * @param component The triconnected component to inspect.
     * @return True if the constraint pattern matches.
     */
    bool hasOneAngleAndTwoDistanceConstraints(const ConstraintGraph& component)
    {
        int angleConstraintCount = 0;
        int distanceConstraintCount = 0;

        const auto& constraintMap = component.getConstraintMap();
        for (const auto& [edgeId, constraint] : constraintMap) {
            if (component.isVirtualEdge(edgeId)) {
                continue;
            }
            if (constraint->template isConstraintType<AngleConstraint>()) {
                ++angleConstraintCount;
            } else if (constraint
                           ->template isConstraintType<DistanceConstraint>()) {
                ++distanceConstraintCount;
            }
        }

        return angleConstraintCount == 1 && distanceConstraintCount == 2;
    }

    /**
     * @brief Check that non-virtual constraints are exactly
     *        1 AngleConstraint and 1 DistanceConstraint.
     *
     * Used by the partially-solved solver to validate the
     * expected constraint pattern: angle between the fixed
     * and free lines, plus distance from the fixed point
     * to the free line.
     *
     * @param component The triconnected component to inspect.
     * @return True if the constraint pattern matches.
     */
    bool hasOneAngleAndOneDistanceConstraint(const ConstraintGraph& component)
    {
        int angleConstraintCount = 0;
        int distanceConstraintCount = 0;

        const auto& constraintMap = component.getConstraintMap();
        for (const auto& [edgeId, constraint] : constraintMap) {
            if (component.isVirtualEdge(edgeId)) {
                continue;
            }
            if (constraint->template isConstraintType<AngleConstraint>()) {
                ++angleConstraintCount;
            } else if (constraint
                           ->template isConstraintType<DistanceConstraint>()) {
                ++distanceConstraintCount;
            }
        }

        return angleConstraintCount == 1 && distanceConstraintCount == 1;
    }

    /**
     * @brief Reconstruct line segment endpoints in solver space.
     *
     * Given the solved infinite line (by its unit normal and
     * perpendicular offset) and two reference points (in solver
     * space), projects those points onto the line and extends
     * the segment to match the original canvas line length.
     *
     * @param referencePoint1 First solver-space reference point
     *        for projection (e.g., the constraining point).
     * @param referencePoint2 Second solver-space reference point
     *        for projection (e.g., intersection with another line
     *        or the line's midpoint).
     * @param normalX X component of the solved unit normal.
     * @param normalY Y component of the solved unit normal.
     * @param perpendicularOffset The line's perpendicular offset
     *        from the origin.
     * @param canvasLineLength Length of the original canvas line.
     * @return Pair of (solverP1, solverP2) line endpoints.
     */
    std::pair<Eigen::Vector2d, Eigen::Vector2d> reconstructLineEndpoints(
        const Eigen::Vector2d& referencePoint1,
        const Eigen::Vector2d& referencePoint2, double normalX, double normalY,
        double perpendicularOffset, double canvasLineLength)
    {
        Eigen::Vector2d normal { normalX, normalY };

        // Project both reference points onto the solved line
        auto projectOntoLine
            = [&normal, perpendicularOffset](const Eigen::Vector2d& point) {
                  double signedDistanceFromLine
                      = normal.dot(point) - perpendicularOffset;
                  return point - signedDistanceFromLine * normal;
              };

        Eigen::Vector2d projection1 = projectOntoLine(referencePoint1);
        Eigen::Vector2d projection2 = projectOntoLine(referencePoint2);

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
// ZeroFixedLLPAngleTriangleSolver
// ================================================================

bool ZeroFixedLLPAngleTriangleSolver::matches(const ConstraintGraph& component)
{
    if (component.nodeCount() != 3 || component.edgeCount() != 3) {
        return false;
    }

    if (component.numberOfSolvedElements() != 0) {
        return false;
    }

    auto [pointCount, lineCount] = countElementTypes(component);
    if (pointCount != 1 || lineCount != 2) {
        return false;
    }

    return hasOneAngleAndTwoDistanceConstraints(component);
}

SolveResult ZeroFixedLLPAngleTriangleSolver::solve(ConstraintGraph& component)
{
    const auto& nodesToElementsMap = component.getElementMap();

    // --------------------------------------------------------
    // Identify the two lines and the point
    // --------------------------------------------------------
    std::shared_ptr<Element> line1Element;
    std::shared_ptr<Element> line2Element;
    std::shared_ptr<Element> pointElement;

    ConstraintGraph::NodeIdType line1Node {};
    ConstraintGraph::NodeIdType line2Node {};
    ConstraintGraph::NodeIdType pointNode {};

    for (const auto& [node, element] : nodesToElementsMap) {
        if (element->template isElementType<Point>()) {
            pointElement = element;
            pointNode = node;
        } else if (element->template isElementType<Line>()) {
            if (!line1Element) {
                line1Element = element;
                line1Node = node;
            } else {
                line2Element = element;
                line2Node = node;
            }
        }
    }

    // --------------------------------------------------------
    // Retrieve the constraints
    // --------------------------------------------------------

    // The angle constraint is between the two lines
    auto lineToLineAngleConstraint
        = component.getConstraintBetweenNodes(line1Node, line2Node);
    double angleBetweenLines
        = lineToLineAngleConstraint->getConstraintValue().value();

    // Distance constraints from the point to each line
    auto pointToLine1Constraint
        = component.getConstraintBetweenNodes(pointNode, line1Node);
    auto pointToLine2Constraint
        = component.getConstraintBetweenNodes(pointNode, line2Node);

    double pointToLine1Distance
        = pointToLine1Constraint->getConstraintValue().value();
    double pointToLine2Distance
        = pointToLine2Constraint->getConstraintValue().value();

    const auto& line1Data = line1Element->getElement<Line>();
    const auto& line2Data = line2Element->getElement<Line>();
    const auto& pointData = pointElement->getElement<Point>();

    // --------------------------------------------------------
    // Step 1: Fix line 1 on the X-axis through the origin.
    //
    // This consumes 2 rigid-body DOF: the line's direction
    // (rotation) and its perpendicular offset (one translation
    // component).
    // --------------------------------------------------------
    Eigen::Vector2d canvasLine1Direction
        = line1Data.canvasP2 - line1Data.canvasP1;
    double canvasLine1Length = canvasLine1Direction.norm();

    Eigen::Vector2d anchorLine1P1 { -canvasLine1Length / 2.0, 0.0 };
    Eigen::Vector2d anchorLine1P2 { canvasLine1Length / 2.0, 0.0 };
    line1Element->updateElementPosition(anchorLine1P1, anchorLine1P2);

    // --------------------------------------------------------
    // Step 2: Place the point at (0, signedDist) directly
    // above/below the origin.
    //
    // This pins the point's along-line projection at the
    // origin, consuming the last rigid-body DOF (translation
    // along line 1).
    // --------------------------------------------------------
    double canvasSignedDistancePointToLine1 = signedDistanceToLine(
        pointData.canvasPosition, line1Data.canvasP1, line1Data.canvasP2);

    auto signOf = [](double x) -> double { return (x > 0.0) ? 1.0 : -1.0; };

    double signedDistancePointToLine1
        = signOf(canvasSignedDistancePointToLine1) * pointToLine1Distance;

    Eigen::Vector2d anchorPointPosition { 0.0, signedDistancePointToLine1 };
    pointElement->updateElementPosition(anchorPointPosition);

    // --------------------------------------------------------
    // Step 3: Solve line 2's unit normal (nx, ny) via
    // Newton-Raphson.
    //
    // The anchored line 1 has direction (1, 0) and length
    // equal to the canvas length. The equations are:
    //   lineNormalAngleConstraint: -ny*fdx + nx*fdy
    //       - fixedLineLength * cos(angle) = 0
    //   unitNormalConstraint: nx^2 + ny^2 - 1 = 0
    // --------------------------------------------------------

    // Anchored line 1 direction (in solver space)
    Eigen::Vector2d anchorLine1Direction = anchorLine1P2 - anchorLine1P1;
    double anchorLine1Length = anchorLine1Direction.norm();

    double cosineOfAngle = std::cos(angleBetweenLines);

    auto angleEquation
        = Equations::lineNormalAngleConstraint(anchorLine1Direction.x(),
            anchorLine1Direction.y(), anchorLine1Length, cosineOfAngle);

    auto unitConstraintEquation = Equations::unitNormalConstraint();

    // Use the canvas line 2 normal as initial guesses
    Eigen::Vector2d canvasLine2Direction
        = line2Data.canvasP2 - line2Data.canvasP1;
    double canvasLine2Length = canvasLine2Direction.norm();
    Eigen::Vector2d canvasLine2Normal { -canvasLine2Direction.y()
            / canvasLine2Length,
        canvasLine2Direction.x() / canvasLine2Length };

    std::array<Eigen::Vector2d, 2> normalInitialGuesses { canvasLine2Normal,
        -canvasLine2Normal };

    auto candidateNormals = Equations::solve2D(
        angleEquation, unitConstraintEquation, normalInitialGuesses);

    // --------------------------------------------------------
    // Step 4: Disambiguate by comparing the angular
    // orientation (cross product sign) with the canvas layout.
    // If flipOrientation is set, negate one direction to pick
    // the opposite side.
    // --------------------------------------------------------
    Eigen::Vector2d canvasFreeLineDirection
        = line2Data.canvasP2 - line2Data.canvasP1;

    const auto* angleData
        = lineToLineAngleConstraint->getConstraintAs<AngleConstraint>();
    if (angleData != nullptr && angleData->flipOrientation) {
        canvasFreeLineDirection = -canvasFreeLineDirection;
    }

    Eigen::Vector2d chosenNormal
        = pickLineNormalByAngleOrientation(canvasLine1Direction,
            canvasFreeLineDirection, candidateNormals[0], candidateNormals[1]);

    // --------------------------------------------------------
    // Step 5: Compute line 2's perpendicular offset from the
    // point-to-line-2 distance.
    //
    // The signed perpendicular distance from the point to
    // line 2 is: n . P - offset = signedDist
    // Therefore: offset = n . P - signedDist
    // --------------------------------------------------------
    double canvasSignedDistancePointToLine2 = signedDistanceToLine(
        pointData.canvasPosition, line2Data.canvasP1, line2Data.canvasP2);

    double signedDistancePointToLine2
        = signOf(canvasSignedDistancePointToLine2) * pointToLine2Distance;

    double line2PerpendicularOffset
        = chosenNormal.dot(anchorPointPosition) - signedDistancePointToLine2;

    // --------------------------------------------------------
    // Step 6: Reconstruct line 2 endpoints in solver space.
    //
    // Use the anchor point and the origin (a point on line 1)
    // as reference points for the projection.
    // --------------------------------------------------------
    Eigen::Vector2d originPoint { 0.0, 0.0 };

    auto [solverLine2P1, solverLine2P2] = reconstructLineEndpoints(
        anchorPointPosition, originPoint, chosenNormal.x(), chosenNormal.y(),
        line2PerpendicularOffset, canvasLine2Length);

    line2Element->updateElementPosition(solverLine2P1, solverLine2P2);

    std::cerr << std::format("LLPAngle Anchor: L1=({},{})--({},{}), "
                             "L2=({},{})--({},{}), P=({},{})\n",
        anchorLine1P1.x(), anchorLine1P1.y(), anchorLine1P2.x(),
        anchorLine1P2.y(), solverLine2P1.x(), solverLine2P1.y(),
        solverLine2P2.x(), solverLine2P2.y(), anchorPointPosition.x(),
        anchorPointPosition.y());

    return SolveResult::success();
}

// ================================================================
// FixedLineAndPointFreeLineSolver
// ================================================================

bool FixedLineAndPointFreeLineSolver::matches(const ConstraintGraph& component)
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

    // Exactly one Line must be solved, one Line unsolved,
    // and the Point must be solved
    int solvedLineCount = 0;
    int unsolvedLineCount = 0;
    bool pointIsSolved = false;

    for (const auto& element : component.getElements()) {
        if (element->template isElementType<Line>()) {
            if (element->isElementSet()) {
                ++solvedLineCount;
            } else {
                ++unsolvedLineCount;
            }
        } else if (element->template isElementType<Point>()) {
            pointIsSolved = element->isElementSet();
        }
    }

    if (solvedLineCount != 1 || unsolvedLineCount != 1 || !pointIsSolved) {
        return false;
    }

    return hasOneAngleAndOneDistanceConstraint(component);
}

SolveResult FixedLineAndPointFreeLineSolver::solve(ConstraintGraph& component)
{
    const auto& nodesToElementsMap = component.getElementMap();

    // --------------------------------------------------------
    // Identify the fixed line, fixed point, and free line
    // --------------------------------------------------------
    std::shared_ptr<Element> fixedLineElement;
    std::shared_ptr<Element> freeLineElement;
    std::shared_ptr<Element> fixedPointElement;

    ConstraintGraph::NodeIdType fixedLineNode {};
    ConstraintGraph::NodeIdType freeLineNode {};
    ConstraintGraph::NodeIdType fixedPointNode {};

    for (const auto& [node, element] : nodesToElementsMap) {
        if (element->template isElementType<Point>()) {
            fixedPointElement = element;
            fixedPointNode = node;
        } else if (element->template isElementType<Line>()) {
            if (element->isElementSet()) {
                fixedLineElement = element;
                fixedLineNode = node;
            } else {
                freeLineElement = element;
                freeLineNode = node;
            }
        }
    }

    // --------------------------------------------------------
    // Retrieve the constraints
    // --------------------------------------------------------

    // The angle constraint is between the two lines
    auto lineToLineAngleConstraint
        = component.getConstraintBetweenNodes(fixedLineNode, freeLineNode);
    double angleBetweenLines
        = lineToLineAngleConstraint->getConstraintValue().value();

    // The distance constraint is from the fixed point to the
    // free line
    auto pointToFreeLineConstraint
        = component.getConstraintBetweenNodes(fixedPointNode, freeLineNode);
    double pointToFreeLineDistance
        = pointToFreeLineConstraint->getConstraintValue().value();

    const auto& fixedLineData = fixedLineElement->getElement<Line>();
    const auto& freeLineData = freeLineElement->getElement<Line>();
    const auto& fixedPointData = fixedPointElement->getElement<Point>();

    // --------------------------------------------------------
    // Step 1: Solve the free line's unit normal (nx, ny) via
    // Newton-Raphson.
    //
    // The fixed line's direction is taken from its solved
    // endpoints (solver space). The equations are:
    //   lineNormalAngleConstraint: -ny*fdx + nx*fdy
    //       - fixedLineLength * cos(angle) = 0
    //   unitNormalConstraint: nx^2 + ny^2 - 1 = 0
    // --------------------------------------------------------
    Eigen::Vector2d fixedLineDirection = fixedLineData.p2 - fixedLineData.p1;
    double fixedLineLength = fixedLineDirection.norm();

    double cosineOfAngle = std::cos(angleBetweenLines);

    auto angleEquation
        = Equations::lineNormalAngleConstraint(fixedLineDirection.x(),
            fixedLineDirection.y(), fixedLineLength, cosineOfAngle);

    auto unitConstraintEquation = Equations::unitNormalConstraint();

    // Use the canvas free line normal as initial guesses
    Eigen::Vector2d canvasFreeLineDirection
        = freeLineData.canvasP2 - freeLineData.canvasP1;
    double canvasFreeLineLength = canvasFreeLineDirection.norm();
    Eigen::Vector2d canvasFreeLineNormal { -canvasFreeLineDirection.y()
            / canvasFreeLineLength,
        canvasFreeLineDirection.x() / canvasFreeLineLength };

    std::array<Eigen::Vector2d, 2> normalInitialGuesses { canvasFreeLineNormal,
        -canvasFreeLineNormal };

    auto candidateNormals = Equations::solve2D(
        angleEquation, unitConstraintEquation, normalInitialGuesses);

    // --------------------------------------------------------
    // Step 2: Disambiguate by comparing the angular
    // orientation (cross product sign) with the canvas layout.
    // If flipOrientation is set, negate one direction to pick
    // the opposite side.
    // --------------------------------------------------------
    Eigen::Vector2d canvasFixedLineDirection
        = fixedLineData.canvasP2 - fixedLineData.canvasP1;

    const auto* angleData
        = lineToLineAngleConstraint->getConstraintAs<AngleConstraint>();
    if (angleData != nullptr && angleData->flipOrientation) {
        canvasFreeLineDirection = -canvasFreeLineDirection;
    }

    Eigen::Vector2d chosenNormal
        = pickLineNormalByAngleOrientation(canvasFixedLineDirection,
            canvasFreeLineDirection, candidateNormals[0], candidateNormals[1]);

    // --------------------------------------------------------
    // Step 3: Compute the free line's perpendicular offset
    // from the point-to-free-line distance.
    //
    // The signed perpendicular distance from the fixed point
    // to the free line is: n . P - offset = signedDist
    // Therefore: offset = n . P - signedDist
    //
    // The sign is determined from the canvas layout.
    // --------------------------------------------------------
    double canvasSignedDistancePointToFreeLine
        = signedDistanceToLine(fixedPointData.canvasPosition,
            freeLineData.canvasP1, freeLineData.canvasP2);

    auto signOf = [](double x) -> double { return (x > 0.0) ? 1.0 : -1.0; };

    double signedDistancePointToFreeLine
        = signOf(canvasSignedDistancePointToFreeLine) * pointToFreeLineDistance;

    double freeLinePerpendicularOffset
        = chosenNormal.dot(fixedPointData.position)
        - signedDistancePointToFreeLine;

    // --------------------------------------------------------
    // Step 4: Reconstruct free line endpoints in solver space.
    //
    // Use the fixed point and the fixed line's midpoint as
    // reference points for the projection.
    // --------------------------------------------------------
    Eigen::Vector2d fixedLineMidpoint = fixedLineData.midpoint();

    auto [solverFreeLineP1, solverFreeLineP2] = reconstructLineEndpoints(
        fixedPointData.position, fixedLineMidpoint, chosenNormal.x(),
        chosenNormal.y(), freeLinePerpendicularOffset, canvasFreeLineLength);

    freeLineElement->updateElementPosition(solverFreeLineP1, solverFreeLineP2);

    std::cerr << std::format("FixedLineAndPoint: fixedLine=({},{})--({},{}), "
                             "fixedPt=({},{}), freeLine=({},{})--({},{})\n",
        fixedLineData.p1.x(), fixedLineData.p1.y(), fixedLineData.p2.x(),
        fixedLineData.p2.y(), fixedPointData.position.x(),
        fixedPointData.position.y(), solverFreeLineP1.x(), solverFreeLineP1.y(),
        solverFreeLineP2.x(), solverFreeLineP2.y());

    return SolveResult::success();
}

} // namespace Gcs::Solvers
