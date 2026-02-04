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

} // namespace Gcs::Solvers

#endif // HEURISTICS_HPP
