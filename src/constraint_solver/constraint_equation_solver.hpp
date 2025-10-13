#ifndef CONSTRAINT_EQUATION_SOLVER_HPP
#define CONSTRAINT_EQUATION_SOLVER_HPP

#include "elements.hpp"
#include <cmath>
#include <string>
#include <format>

namespace Solver {
struct Coordinates2D {
    double x;
    double y;

    void normalize()
    {
        float len = std::sqrt(x * x + y * y);
        x /= len;
        y /= len;
    }
};

struct TriangleOrientationHeuristicData {
    Coordinates2D fixedPointPone;
    Coordinates2D fixedPointPtwo;
    Coordinates2D freePointPthree;

    std::string toString() const
    {
        return std::format("fixedPointPone: ({},{}), fixedPointPtwo: ({},{}), "
                           "freePointPthree: ({},{})",
            fixedPointPone.x, fixedPointPone.y, fixedPointPtwo.x,
            fixedPointPtwo.y, freePointPthree.x, freePointPthree.y);
    }
};

struct PointOnLineOrientationHeuristicData {
    Coordinates2D fixedLinePone;
    Coordinates2D fixedLinePtwo;
    Coordinates2D fixedPointPone;
    Coordinates2D freePointPtwo;
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
 * @param heuristicInfo Information from the user canvas to make initial guess
 * heuristics for Newton-Raphson
 */
std::tuple<Coordinates2D, Coordinates2D, Coordinates2D>
calculatePointToPointDistanceTriangle(const double xToYDistance,
    const double xToZDistance, const double yToZDistance,
    TriangleOrientationHeuristicData heuristicInfo);

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
 * @param heuristicInfo Information from the user canvas to make initial guess
 * heuristics for Newton-Raphson
 */
Coordinates2D calculatePointToPointDistanceTriangleFromTwoFixedPoints(
    const Coordinates2D& p1, const Coordinates2D& p2, const double distanceP2P3,
    const double distanceP1P3, TriangleOrientationHeuristicData heuristicInfo);

Coordinates2D calculatePointToPointDistanceTriangleOnLine(
    std::pair<Coordinates2D, Coordinates2D> fixedLine,
    Coordinates2D fixedPointOnLine, double distanceFromFixedPoint,
    PointOnLineOrientationHeuristicData heuristicInfo);

}; // namespace Solver

#endif // CONSTRAINT_EQUATION_SOLVER_HPP
