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

    Coordinates2D chooseValidOutput(
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

    autodiff::dual f(
        autodiff::dual xCoord, autodiff::dual yCoord, const autodiff::dual& d3)
    {
        return pow(xCoord, 2) + pow(yCoord, 2) - pow(d3, 2);
    }
    autodiff::dual g(autodiff::dual xCoord, autodiff::dual yCoord,
        const autodiff::dual& d1, const autodiff::dual& d2)
    {
        return pow(d1 - xCoord, 2) + pow(yCoord, 2) - pow(d2, 2);
    }
}

// NOTE: this is hardcore code duplication, later when time is given, this
// should be generalized
std::tuple<Coordinates2D, Coordinates2D, Coordinates2D>
calculatePointToPointDistanceTriangle(const double xToYDistance,
    const double xToZDistance, const double yToZDistance,
    TriangleOrientationHeuristicData heuristicInfo)
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

    // Variables are given a starting non-zero value, later this could be more
    // sophisticated

    for (auto& variableValues : possibleVariableValues) {
        Dual2DColVector prevValues = { 0, 0 };

        Eigen::Matrix2d jacobian;
        Eigen::Vector2d evaluatedFunctionValues;
        Eigen::Vector2d updatedVariableValues;

        for (auto i = 0; i < MAXIMUM_ITERATIONS; i++) {
            jacobian(0, 0) = derivative(f, wrt(variableValues.x()),
                at(variableValues.x(), variableValues.y(), xToZDistance));
            jacobian(0, 1) = derivative(f, wrt(variableValues.y()),
                at(variableValues.x(), variableValues.y(), xToZDistance));
            jacobian(1, 0) = derivative(g, wrt(variableValues.x()),
                at(variableValues.x(), variableValues.y(), xToYDistance,
                    yToZDistance));
            jacobian(1, 1) = derivative(g, wrt(variableValues.y()),
                at(variableValues.x(), variableValues.y(), xToYDistance,
                    yToZDistance));

            evaluatedFunctionValues = Eigen::Vector2d {
                -(f(variableValues.x(), variableValues.y(), xToZDistance).val),
                -(g(variableValues.x(), variableValues.y(), xToYDistance,
                    yToZDistance)
                        .val)
            };

            // Solving J_f(x_k)s_k = -f(x_k)
            updatedVariableValues
                = jacobian.colPivHouseholderQr().solve(evaluatedFunctionValues);
            if (abs(prevValues.x() - variableValues.x()) < ERROR
                && abs(prevValues.y() - variableValues.y()) < ERROR) {
                break;
            }
            // Claude suggestion, not sure about it
            //  if (abs(updatedVariableValues[0]) < ERROR
            //              && abs(updatedVariableValues[1]) < ERROR)
            prevValues = variableValues;

            // Calculating x_k+1 = x_k + s_k
            variableValues.x() += dual(updatedVariableValues[0]);
            variableValues.y() += dual(updatedVariableValues[1]);
        }
    }

    Coordinates2D origin { 0, 0 };
    Coordinates2D distanceOnXFromOrigin { xToYDistance, 0 };

    return { origin, distanceOnXFromOrigin,
        Coordinates2D { chooseValidOutput(heuristicInfo,
            { origin, distanceOnXFromOrigin,
                { possibleVariableValues.front().x().val,
                    possibleVariableValues.front().y().val } },
            { origin, distanceOnXFromOrigin,
                { possibleVariableValues.back().x().val,
                    possibleVariableValues.back().y().val } }) } };
}

namespace {
    struct FunctionsForSolvingOneUknownPointDistance {
        using dual = autodiff::dual;
        static dual f(const dual& x1, const dual& y1, const dual& d3,
            dual xCoord, dual yCoord)
        {
            return pow(xCoord - x1, 2) + pow(yCoord - y1, 2) - pow(d3, 2);
        }
        static dual g(const dual& x2, const dual& y2, const dual& d2,
            dual xCoord, dual yCoord)
        {
            return pow(xCoord - x2, 2) + pow(yCoord - y2, 2) - pow(d2, 2);
        }
    };
}

Coordinates2D calculatePointToPointDistanceTriangleFromTwoFixedPoints(
    const Coordinates2D& p1, const Coordinates2D& p2, const double distanceP2P3,
    const double distanceP1P3, TriangleOrientationHeuristicData heuristicInfo)
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

    FunctionsForSolvingOneUknownPointDistance functions;

    for (auto& variableValues : possibleVariableValues) {

        // Variables are given a starting non-zero value, later this could be
        // more sophisticated
        Dual2DColVector prevValues = { 0, 0 };

        Eigen::Matrix2d jacobian;
        Eigen::Vector2d evaluatedFunctionValues;
        Eigen::Vector2d updatedVariableValues;

        for (auto i = 0; i < MAXIMUM_ITERATIONS; i++) {
            jacobian(0, 0) = derivative(functions.f, wrt(variableValues.x()),
                at(p1.x, p1.y, distanceP1P3, variableValues.x(),
                    variableValues.y()));
            jacobian(0, 1) = derivative(functions.f, wrt(variableValues.y()),
                at(p1.x, p1.y, distanceP1P3, variableValues.x(),
                    variableValues.y()));
            jacobian(1, 0) = derivative(functions.g, wrt(variableValues.x()),
                at(p2.x, p2.y, distanceP2P3, variableValues.x(),
                    variableValues.y()));
            jacobian(1, 1) = derivative(functions.g, wrt(variableValues.y()),
                at(p2.x, p2.y, distanceP2P3, variableValues.x(),
                    variableValues.y()));

            evaluatedFunctionValues = Eigen::Vector2d {
                -(functions
                        .f(p1.x, p1.y, distanceP1P3, variableValues.x(),
                            variableValues.y())
                        .val),
                -(functions
                        .g(p2.x, p2.y, distanceP2P3, variableValues.x(),
                            variableValues.y())
                        .val)
            };

            // Solving J_f(x_k)s_k = -f(x_k)
            updatedVariableValues
                = jacobian.colPivHouseholderQr().solve(evaluatedFunctionValues);
            if (abs(prevValues.x() - variableValues.x()) < ERROR
                && abs(prevValues.y() - variableValues.y()) < ERROR) {
                break;
            }
            // Claude suggestion, not sure about it
            //  if (abs(updatedVariableValues[0]) < ERROR
            //             && abs(updatedVariableValues[1]) < ERROR)

            prevValues = variableValues;

            // Calculating x_k+1 = x_k + s_k
            variableValues.x() += dual(updatedVariableValues[0]);
            variableValues.y() += dual(updatedVariableValues[1]);
        }
    }

    return Coordinates2D { chooseValidOutput(heuristicInfo,
        { p1, p2,
            { possibleVariableValues.front().x().val,
                possibleVariableValues.front().y().val } },
        { p1, p2,
            { possibleVariableValues.back().x().val,
                possibleVariableValues.back().y().val } }) };
}
} // namespace Solver
