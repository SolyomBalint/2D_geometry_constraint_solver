#ifndef CONSTRAINT_EQUATION_SOLVER_HPP
#define CONSTRAINT_EQUATION_SOLVER_HPP

#include <vector>

namespace Solver {
struct Coordinates2D {
    double x;
    double y;
};

/**
 * @brief Calculates the coordinates of edges of a triangle, defined by the
 * input constraints using the Newton-Raphson method
 *
 * @note a modified version of the Newton-Raphson is used where we calculate the
 * equation: J_f(x_k)s_k = -f(x_k) is solved and the next iteration will use:
 * x_k+1 = x_k + s_k
 *
 * @param xToYDistance The distance between points x and y
 * @param xToZDistance The distance between points x and z
 * @param yToZDistance The distance between points y and z
 */
std::tuple<Coordinates2D, Coordinates2D, Coordinates2D>
calculatePointToPointDistanceTriangle(const double xToYDistance,
    const double xToZDistance, const double yToZDistance);

/**
 * @brief Calculate the values of a third point based on two constant points and
 * distance constraints using the Newton-Raphson method in the form of
 * J_f(x_k)s_k = -f(x_k)
 *
 * @param p1 coordinates of the first constant point
 * @param p2 coordinates of the first constant point
 * @param distanceP2P3 distance constraint between the p2 and the third point we
 * want to calculate (d2)
 * @param distanceP1P3 distance constraint between the p1 and the third point we
 * want to calculate (d3)
 */
Coordinates2D calculatePointToPointDistanceTriangleFromTwoFixedPoints(
    const Coordinates2D& p1, const Coordinates2D& p2, const double distanceP2P3,
    const double distanceP1P3);

}; // namespace Solver

#endif // CONSTRAINT_EQUATION_SOLVER_HPP
