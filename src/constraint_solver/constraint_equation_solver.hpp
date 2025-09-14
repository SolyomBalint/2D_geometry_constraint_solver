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
std::vector<Coordinates2D> calculatePointToPointDistanceTriangle(
    const double xToYDistance, const double xToZDistance,
    const double yToZDistance);
}; // namespace Solver

#endif // CONSTRAINT_EQUATION_SOLVER_HPP
