#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

// Thirdparty headers
#include <Eigen/Core>

namespace Gcs::Solvers {

/**
 * @brief Compute the signed area (2D cross product) of triangle ABC.
 *
 * @param a First vertex.
 * @param b Second vertex.
 * @param c Third vertex.
 * @return Positive if A->B->C is counter-clockwise, negative if
 *         clockwise, zero if collinear.
 */
inline double triangleOrientation(const Eigen::Vector2d& a,
    const Eigen::Vector2d& b, const Eigen::Vector2d& c)
{
    return (b.x() - a.x()) * (c.y() - a.y())
        - (b.y() - a.y()) * (c.x() - a.x());
}

/**
 * @brief Pick the candidate whose triangle orientation matches the
 *        original canvas layout.
 *
 * Newton-Raphson produces two mirror-image solutions for a triangle
 * vertex. This function picks the one that preserves the orientation
 * (clockwise vs counter-clockwise) of the user's sketch.
 *
 * @param canvasA Canvas position of the first fixed point.
 * @param canvasB Canvas position of the second fixed point.
 * @param canvasFree Canvas position of the free point (user sketch).
 * @param fixedA Solver-computed position of the first fixed point.
 * @param fixedB Solver-computed position of the second fixed point.
 * @param candidate0 First Newton-Raphson solution for the free point.
 * @param candidate1 Second Newton-Raphson solution for the free point.
 * @return Whichever candidate preserves the canvas orientation.
 */
inline Eigen::Vector2d pickByTriangleOrientation(const Eigen::Vector2d& canvasA,
    const Eigen::Vector2d& canvasB, const Eigen::Vector2d& canvasFree,
    const Eigen::Vector2d& fixedA, const Eigen::Vector2d& fixedB,
    const Eigen::Vector2d& candidate0, const Eigen::Vector2d& candidate1)
{
    double canvasOri = triangleOrientation(canvasA, canvasB, canvasFree);
    double sol0Ori = triangleOrientation(fixedA, fixedB, candidate0);

    auto sign = [](double x) { return (x > 0) - (x < 0); };

    return (sign(canvasOri) == sign(sol0Ori)) ? candidate0 : candidate1;
}

/**
 * @brief Pick the candidate point-on-line solution that preserves
 *        the relative orientation from the canvas layout.
 *
 * For a point constrained to lie on a line, Newton-Raphson produces
 * two solutions (one on each side of a reference point along the
 * line). This function picks the one that matches the canvas
 * orientation using the dot product of direction vectors.
 *
 * @param canvasLineP1 Canvas position of line endpoint 1.
 * @param canvasLineP2 Canvas position of line endpoint 2.
 * @param canvasFixedPoint Canvas position of the fixed reference point.
 * @param canvasFreePoint Canvas position of the free point (sketch).
 * @param solverFixedPoint Solver position of the fixed reference point.
 * @param candidate0 First Newton-Raphson solution.
 * @param candidate1 Second Newton-Raphson solution.
 * @return Whichever candidate preserves the canvas orientation.
 */
inline Eigen::Vector2d pickByLineOrientation(
    const Eigen::Vector2d& canvasLineP1, const Eigen::Vector2d& canvasLineP2,
    const Eigen::Vector2d& canvasFixedPoint,
    const Eigen::Vector2d& canvasFreePoint,
    const Eigen::Vector2d& solverFixedPoint, const Eigen::Vector2d& candidate0,
    const Eigen::Vector2d& candidate1)
{
    Eigen::Vector2d lineDir = canvasLineP2 - canvasLineP1;
    Eigen::Vector2d canvasPointDir = canvasFixedPoint - canvasFreePoint;

    double canvasOri = canvasPointDir.dot(lineDir);

    Eigen::Vector2d solverPointDir = solverFixedPoint - candidate0;
    double sol0Ori = solverPointDir.dot(lineDir);

    auto sign = [](double x) { return (x > 0) - (x < 0); };

    return (sign(canvasOri) == sign(sol0Ori)) ? candidate0 : candidate1;
}

/**
 * @brief Compute the signed perpendicular distance from a point to
 *        a line defined by two endpoints.
 *
 * Uses the cross-product formula:
 * @c (lineP2 - lineP1) x (point - lineP1) / |lineP2 - lineP1|
 *
 * Positive means the point is on the left side of the line
 * (counter-clockwise from the line direction), negative means
 * right side.
 *
 * @param point The query point.
 * @param lineP1 First endpoint of the line.
 * @param lineP2 Second endpoint of the line.
 * @return Signed perpendicular distance.
 */
inline double signedDistanceToLine(const Eigen::Vector2d& point,
    const Eigen::Vector2d& lineP1, const Eigen::Vector2d& lineP2)
{
    Eigen::Vector2d lineDirection = lineP2 - lineP1;
    double lineLength = lineDirection.norm();
    Eigen::Vector2d pointToLineP1 = point - lineP1;

    // Cross product: lineDirection x pointToLineP1
    double crossProduct = lineDirection.x() * pointToLineP1.y()
        - lineDirection.y() * pointToLineP1.x();

    return crossProduct / lineLength;
}

/**
 * @brief Pick the candidate line whose signed-distance pattern
 *        matches the canvas layout.
 *
 * Each candidate line is described by its unit normal @c (nx, ny)
 * and perpendicular offset @c p. For each candidate, the signed
 * distances from the two fixed points are compared against the
 * canvas signed distances.
 *
 * @param canvasSignedDistanceFromPoint1 Signed distance from
 *        canvas point 1 to the canvas line.
 * @param canvasSignedDistanceFromPoint2 Signed distance from
 *        canvas point 2 to the canvas line.
 * @param candidate0 First Newton-Raphson solution @c (nx, ny).
 * @param candidate1 Second Newton-Raphson solution @c (nx, ny).
 * @param fixedPoint1 Solver position of the first fixed point.
 * @param fixedPoint2 Solver position of the second fixed point.
 * @param perpendicularOffset1 Perpendicular offset @c p for
 *        candidate 0 (computed as @c n . P1 - signedDist1).
 * @param perpendicularOffset2 Perpendicular offset @c p for
 *        candidate 1.
 * @return The candidate @c (nx, ny) and offset @c p that matches
 *         the canvas sign pattern, as a tuple @c (nx, ny, p).
 */
inline std::tuple<double, double, double> pickLineBySignedDistances(
    double canvasSignedDistanceFromPoint1,
    double canvasSignedDistanceFromPoint2, const Eigen::Vector2d& candidate0,
    const Eigen::Vector2d& candidate1, const Eigen::Vector2d& fixedPoint1,
    const Eigen::Vector2d& fixedPoint2, double perpendicularOffset0,
    double perpendicularOffset1)
{
    auto sign = [](double x) { return (x > 0) - (x < 0); };

    int canvasSign1 = sign(canvasSignedDistanceFromPoint1);
    int canvasSign2 = sign(canvasSignedDistanceFromPoint2);

    // For candidate 0, compute signed distances from each fixed
    // point to the candidate line:  n . point - p
    double solverSignedDist1FromCandidate0
        = candidate0.dot(fixedPoint1) - perpendicularOffset0;
    double solverSignedDist2FromCandidate0
        = candidate0.dot(fixedPoint2) - perpendicularOffset0;

    int solverSign1 = sign(solverSignedDist1FromCandidate0);
    int solverSign2 = sign(solverSignedDist2FromCandidate0);

    if (solverSign1 == canvasSign1 && solverSign2 == canvasSign2) {
        return { candidate0.x(), candidate0.y(), perpendicularOffset0 };
    }

    return { candidate1.x(), candidate1.y(), perpendicularOffset1 };
}

} // namespace Gcs::Solvers

#endif // HEURISTICS_HPP
