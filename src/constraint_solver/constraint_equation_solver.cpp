#include "constraint_equation_solver.hpp"

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <format>
#include <Eigen/Dense>

namespace Solver {
// Keep internal linkage for now
static constexpr double ERROR = 0.00001;
static constexpr int32_t MAXIMUM_ITERATIONS = 1000;

namespace {

    double triangleOrientation(
        Coordinates2D fixedP1, Coordinates2D fixedP2, Coordinates2D scetchPoint)
    {
        return (fixedP2.x - fixedP1.x) * (scetchPoint.y - fixedP1.y)
            - (fixedP2.y - fixedP1.y) * (scetchPoint.x - fixedP1.x);
    }

    Coordinates2D chooseValidOutputTriangle(
        const TriangleOrientationHeuristicData& canvasCoords,
        const TriangleOrientationHeuristicData& possibleSolution1,
        const TriangleOrientationHeuristicData& possibleSolution2)
    {

        auto canvasOrientation { triangleOrientation(
            canvasCoords.fixedPointPone, canvasCoords.fixedPointPtwo,
            canvasCoords.freePointPthree) };
        auto solutionOneOrientation { triangleOrientation(
            possibleSolution1.fixedPointPone, possibleSolution1.fixedPointPtwo,
            possibleSolution1.freePointPthree) };

        auto sign = [](double x) { return (x > 0) - (x < 0); };

        return sign(canvasOrientation) == sign(solutionOneOrientation)
            ? possibleSolution1.freePointPthree
            : possibleSolution2.freePointPthree;
    }

    Coordinates2D chooseValidOutputPointsOnLine(
        const PointOnLineOrientationHeuristicData& canvasData,
        const PointOnLineOrientationHeuristicData& possibleSolution1,
        const PointOnLineOrientationHeuristicData& possibleSolution2)
    {
        using Dual2DColVector = Eigen::Matrix<double, 2, 1>;
        using Dual2DRowVector = Eigen::Matrix<double, 1, 2>;

        Dual2DColVector lineDirectionVectorCanvas { canvasData.fixedLinePtwo.x
                - canvasData.fixedLinePone.x,
            canvasData.fixedLinePtwo.y - canvasData.fixedLinePone.y };

        Dual2DRowVector pointOrientationVectorCanvas {
            canvasData.fixedPointPone.x - canvasData.freePointPtwo.x,
            canvasData.fixedPointPone.y - canvasData.freePointPtwo.y
        };

        double canvasOrientation
            = pointOrientationVectorCanvas * lineDirectionVectorCanvas;

        Dual2DColVector lineDirectionVector { canvasData.fixedLinePtwo.x
                - canvasData.fixedLinePone.x,
            canvasData.fixedLinePtwo.y - canvasData.fixedLinePone.y };

        Dual2DRowVector pointOrientationVector { canvasData.fixedPointPone.x
                - canvasData.freePointPtwo.x,
            canvasData.fixedPointPone.y - canvasData.freePointPtwo.y };

        double solutionOneOrientation
            = pointOrientationVector * lineDirectionVector;

        auto sign = [](double x) { return (x > 0) - (x < 0); };

        return sign(canvasOrientation) == sign(solutionOneOrientation)
            ? possibleSolution1.freePointPtwo
            : possibleSolution2.freePointPtwo;
    }

    /**
     * @brief Universal Newton-Raphson solver for 2D systems of equations
     *
     * Solves a system of two nonlinear equations f(x,y) = 0 and g(x,y) = 0
     * using the Newton-Raphson method with automatic differentiation.
     *
     * @tparam FuncF Type of the first function object
     * @tparam FuncG Type of the second function object
     * @param f Function object for first equation: f(x,y) = 0
     * @param g Function object for second equation: g(x,y) = 0
     * @return std::array<Eigen::Vector2d, 2> Two possible solutions [x,y]
     */
    template <typename FuncF, typename FuncG>
    std::array<Eigen::Vector2d, 2> solveNewtonRaphson2D(FuncF&& f, FuncG&& g)
    {
        using autodiff::at;
        using autodiff::derivative;
        using autodiff::dual;
        using autodiff::wrt;
        using Dual2DColVector = Eigen::Matrix<dual, 2, 1>;

        std::array<Dual2DColVector, 2> possibleVariableValues {
            (Dual2DColVector() << 5000, 5000).finished(),
            (Dual2DColVector() << -5000, -5000).finished()
        };

        std::array<Eigen::Vector2d, 2> solutions { Eigen::Vector2d(),
            Eigen::Vector2d() };

        for (size_t idx = 0; idx < possibleVariableValues.size(); ++idx) {
            auto& variableValues = possibleVariableValues[idx];
            Dual2DColVector prevValues = { 0, 0 };

            Eigen::Matrix2d jacobian;
            Eigen::Vector2d evaluatedFunctionValues;
            Eigen::Vector2d updatedVariableValues;

            for (auto i = 0; i < MAXIMUM_ITERATIONS; i++) {
                // Compute Jacobian matrix
                jacobian(0, 0) = derivative(f, wrt(variableValues.x()),
                    at(variableValues.x(), variableValues.y()));
                jacobian(0, 1) = derivative(f, wrt(variableValues.y()),
                    at(variableValues.x(), variableValues.y()));
                jacobian(1, 0) = derivative(g, wrt(variableValues.x()),
                    at(variableValues.x(), variableValues.y()));
                jacobian(1, 1) = derivative(g, wrt(variableValues.y()),
                    at(variableValues.x(), variableValues.y()));

                // Evaluate function values
                evaluatedFunctionValues(0)
                    = -(f(variableValues.x(), variableValues.y()).val);
                evaluatedFunctionValues(1)
                    = -(g(variableValues.x(), variableValues.y()).val);

                // Solve J_f(x_k)s_k = -f(x_k)
                updatedVariableValues = jacobian.colPivHouseholderQr().solve(
                    evaluatedFunctionValues);

                // Check convergence
                if (abs(prevValues.x() - variableValues.x()) < ERROR
                    && abs(prevValues.y() - variableValues.y()) < ERROR) {
                    break;
                }

                prevValues = variableValues;

                // Calculate x_k+1 = x_k + s_k
                variableValues.x() += dual(updatedVariableValues[0]);
                variableValues.y() += dual(updatedVariableValues[1]);
            }

            solutions[idx](0) = variableValues.x().val;
            solutions[idx](1) = variableValues.y().val;
        }

        return solutions;
    }
}

std::tuple<Coordinates2D, Coordinates2D, Coordinates2D>
calculatePointToPointDistanceTriangle(const double xToYDistance,
    const double xToZDistance, const double yToZDistance,
    TriangleOrientationHeuristicData heuristicInfo)
{
    using autodiff::dual;

    // Create lambda functions that capture the distance constraints as duals
    dual d1 = xToYDistance;
    dual d2 = yToZDistance;
    dual d3 = xToZDistance;

    auto fLambda = [d3](dual x, dual y) -> dual {
        return pow(x, 2) + pow(y, 2) - pow(d3, 2);
    };

    auto gLambda = [d1, d2](dual x, dual y) -> dual {
        return pow(d1 - x, 2) + pow(y, 2) - pow(d2, 2);
    };

    // Solve using universal Newton-Raphson solver
    auto solutions = solveNewtonRaphson2D(fLambda, gLambda);

    Coordinates2D origin { 0, 0 };
    Coordinates2D distanceOnXFromOrigin { xToYDistance, 0 };

    return { origin, distanceOnXFromOrigin,
        Coordinates2D { chooseValidOutputTriangle(heuristicInfo,
            { origin, distanceOnXFromOrigin,
                { solutions[0][0], solutions[0][1] } },
            { origin, distanceOnXFromOrigin,
                { solutions[1][0], solutions[1][1] } }) } };
}

Coordinates2D calculatePointToPointDistanceTriangleFromTwoFixedPoints(
    const Coordinates2D& p1, const Coordinates2D& p2, const double distanceP2P3,
    const double distanceP1P3, TriangleOrientationHeuristicData heuristicInfo)
{
    using autodiff::dual;

    // Create lambda functions that capture the fixed points and distances as
    // duals
    dual x1 = p1.x;
    dual y1 = p1.y;
    dual x2 = p2.x;
    dual y2 = p2.y;
    dual d1 = distanceP1P3;
    dual d2 = distanceP2P3;

    auto fLambda = [x1, y1, d1](dual x, dual y) -> dual {
        return pow(x - x1, 2) + pow(y - y1, 2) - pow(d1, 2);
    };

    auto gLambda = [x2, y2, d2](dual x, dual y) -> dual {
        return pow(x - x2, 2) + pow(y - y2, 2) - pow(d2, 2);
    };

    // Solve using universal Newton-Raphson solver
    auto solutions = solveNewtonRaphson2D(fLambda, gLambda);

    return Coordinates2D { chooseValidOutputTriangle(heuristicInfo,
        { p1, p2, { solutions[0][0], solutions[0][1] } },
        { p1, p2, { solutions[1][0], solutions[1][1] } }) };
}

Coordinates2D calculatePointToPointDistanceTriangleOnLine(
    std::pair<Coordinates2D, Coordinates2D> fixedLine,
    Coordinates2D fixedPointOnLine, double distanceFromFixedPoint,
    PointOnLineOrientationHeuristicData heuristicInfo)
{
    using autodiff::dual;

    // Fixed point P₁
    dual x1 { fixedPointOnLine.x };
    dual y1 { fixedPointOnLine.y };
    dual pointDistance { distanceFromFixedPoint };

    // Distance constraint: (x - x₁)² + (y - y₁)² = d²
    auto pointsDistanceFunction
        = [x1, y1, pointDistance](dual x, dual y) -> dual {
        return pow(x - x1, 2) + pow(y - y1, 2) - pow(pointDistance, 2);
    };

    // Line defining points A and B
    dual xa { fixedLine.first.x };
    dual ya { fixedLine.first.y };
    dual xb { fixedLine.second.x }; // Changed from x2 to xb for clarity
    dual yb { fixedLine.second.y }; // Changed from y2 to yb for clarity

    // Colinearity constraint: A, B, and (x, y) are colinear
    auto threeColinearPoints = [xa, ya, xb, yb](dual x, dual y) -> dual {
        return (xb - xa) * (y - ya) - (yb - ya) * (x - xa);
    };

    auto solutions
        = solveNewtonRaphson2D(pointsDistanceFunction, threeColinearPoints);
    return Coordinates2D { chooseValidOutputPointsOnLine(heuristicInfo,
        { fixedLine.first, fixedLine.second, fixedPointOnLine,
            { solutions[0][0], solutions[0][1] } },
        { fixedLine.first, fixedLine.second, fixedPointOnLine,
            { solutions[1][0], solutions[1][1] } }) };
}

} // namespace Solver
