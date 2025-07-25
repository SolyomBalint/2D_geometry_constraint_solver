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
static const auto SOLVER_LOGGER = spdlog::stdout_color_mt("SOLVER");
static constexpr double ERROR = 0.00001;
static constexpr int32_t MAXIMUM_ITERATIONS = 1000;

autodiff::dual f(autodiff::dual xCoord, autodiff::dual yCoord, const autodiff::dual& d3)
{
    return pow(xCoord, 2) + pow(yCoord, 2) - pow(d3, 2);
}
autodiff::dual g(autodiff::dual xCoord, autodiff::dual yCoord, const autodiff::dual& d1, const autodiff::dual& d2)
{
    return pow(d1 - xCoord, 2) + pow(-yCoord, 2) - pow(d2, 2);
}

std::vector<Coordinates2D> calculatePointToPointDistanceTriangle(
    const double xToYDistance, const double xToZDistance, const double yToZDistance)
{
    using autodiff::at;
    using autodiff::derivative;
    using autodiff::dual;
    using autodiff::wrt;
    using Dual2DColVector = Eigen::Matrix<dual, 2, 1>;

    // Variables are given a starting non-zero value, later this could be more sophisticated
    Dual2DColVector variableValues = { 1, 1 };
    Dual2DColVector prevValues = { 0, 0 };

    Eigen::Matrix2d jacobian;
    Eigen::Vector2d evaluatedFunctionValues;
    Eigen::Vector2d updatedVariableValues;

    for (auto i = 0; i < MAXIMUM_ITERATIONS; i++) {
        jacobian << derivative(f, wrt(variableValues.x()), at(variableValues.x(), variableValues.y(), xToZDistance)),
            derivative(f, wrt(variableValues.y()), at(variableValues.x(), variableValues.y(), xToZDistance)),
            derivative(
                g, wrt(variableValues.x()), at(variableValues.x(), variableValues.y(), xToYDistance, yToZDistance)),
            derivative(
                g, wrt(variableValues.y()), at(variableValues.x(), variableValues.y(), xToYDistance, yToZDistance));

        evaluatedFunctionValues = Eigen::Vector2d { -(f(variableValues.x(), variableValues.y(), xToZDistance).val),
            -(g(variableValues.x(), variableValues.y(), xToYDistance, yToZDistance).val) };

        // Solving J_f(x_k)s_k = -f(x_k)
        updatedVariableValues = jacobian.colPivHouseholderQr().solve(evaluatedFunctionValues);

        if (abs(prevValues.x() - variableValues.x()) < ERROR && abs(prevValues.y() - variableValues.y()) < ERROR) {
            break;
        }

        prevValues = variableValues;

        // Calculating x_k+1 = x_k + s_k
        variableValues.x() += dual(updatedVariableValues[0]);
        variableValues.y() += dual(updatedVariableValues[1]);
    }

    SOLVER_LOGGER->debug(std::format("The calculated result of the two variable equation system is: x: {}, y: {} \n",
        variableValues.x().val, variableValues.y().val));

    return { Coordinates2D { 0, 0 }, Coordinates2D { xToYDistance, 0 },
        Coordinates2D { variableValues.x().val, variableValues.y().val } };
}
} // namespace Solver
