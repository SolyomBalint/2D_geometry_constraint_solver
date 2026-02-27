#ifndef NEWTON_RAPHSON_HPP
#define NEWTON_RAPHSON_HPP

// General STD/STL headers
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Thirdparty headers
#include <Eigen/Dense>
#include <autodiff/forward/dual.hpp>

namespace Gcs::Equations {

/// @brief Convergence threshold for Newton-Raphson iteration.
static constexpr double CONVERGENCE_THRESHOLD = 0.00001;

/// @brief Maximum number of Newton-Raphson iterations before giving up.
static constexpr int32_t MAXIMUM_ITERATIONS = 1000;

/**
 * @brief Solve a 2-unknown nonlinear system via Newton-Raphson.
 *
 * Finds @c (x, y) such that @c f(x, y) = 0 and @c g(x, y) = 0
 * using the modified Newton-Raphson method where each iteration
 * solves @c J_f(x_k) * s_k = -f(x_k), then updates
 * @c x_{k+1} = x_k + s_k.
 *
 * Tries each provided initial guess, returning a candidate
 * solution per guess. The caller is responsible for choosing the
 * geometrically correct one using a disambiguation heuristic.
 *
 * @tparam FuncF Callable with signature @c (dual x, dual y) -> dual.
 * @tparam FuncG Callable with signature @c (dual x, dual y) -> dual.
 * @param f First equation.
 * @param g Second equation.
 * @param initialGuesses Two starting points for Newton-Raphson.
 * @return Two candidate solutions as @c Eigen::Vector2d.
 */
template <typename FuncF, typename FuncG>
std::array<Eigen::Vector2d, 2> solve2D(
    FuncF&& f, FuncG&& g, const std::array<Eigen::Vector2d, 2>& initialGuesses)
{
    using autodiff::at;
    using autodiff::derivative;
    using autodiff::dual;
    using autodiff::wrt;
    using DualVec = Eigen::Matrix<dual, 2, 1>;

    std::array<Eigen::Vector2d, 2> solutions {};

    for (std::size_t idx = 0; idx < initialGuesses.size(); ++idx) {
        DualVec vars;
        vars.x() = dual(initialGuesses[idx].x());
        vars.y() = dual(initialGuesses[idx].y());

        DualVec prevVars = { 0, 0 };

        Eigen::Matrix2d jacobian;
        Eigen::Vector2d funcValues;
        Eigen::Vector2d step;

        for (int32_t i = 0; i < MAXIMUM_ITERATIONS; ++i) {
            // Compute Jacobian matrix
            jacobian(0, 0)
                = derivative(f, wrt(vars.x()), at(vars.x(), vars.y()));
            jacobian(0, 1)
                = derivative(f, wrt(vars.y()), at(vars.x(), vars.y()));
            jacobian(1, 0)
                = derivative(g, wrt(vars.x()), at(vars.x(), vars.y()));
            jacobian(1, 1)
                = derivative(g, wrt(vars.y()), at(vars.x(), vars.y()));

            // Evaluate function values: -f(x_k)
            funcValues(0) = -(f(vars.x(), vars.y()).val);
            funcValues(1) = -(g(vars.x(), vars.y()).val);

            // Solve J_f(x_k) * s_k = -f(x_k)
            step = jacobian.colPivHouseholderQr().solve(funcValues);

            // Check convergence (compare raw double values)
            if (std::abs(prevVars.x().val - vars.x().val)
                    < CONVERGENCE_THRESHOLD
                && std::abs(prevVars.y().val - vars.y().val)
                    < CONVERGENCE_THRESHOLD) {
                break;
            }

            prevVars = vars;

            // Update: x_{k+1} = x_k + s_k
            vars.x() += dual(step[0]);
            vars.y() += dual(step[1]);
        }

        solutions[idx](0) = vars.x().val;
        solutions[idx](1) = vars.y().val;
    }

    return solutions;
}

/// @brief Default initial guesses for spatial-coordinate unknowns.
inline const std::array<Eigen::Vector2d, 2> DEFAULT_SPATIAL_GUESSES {
    Eigen::Vector2d { 20000.0, 20000.0 }, Eigen::Vector2d { -20000.0, -20000.0 }
};

/**
 * @brief Convenience overload using default initial guesses.
 *
 * Uses symmetric spatial initial guesses @c (2000, 2000) and
 * @c (-2000, -2000), suitable when the unknowns represent
 * point coordinates.
 *
 * @tparam FuncF Callable with signature @c (dual x, dual y) -> dual.
 * @tparam FuncG Callable with signature @c (dual x, dual y) -> dual.
 * @param f First equation.
 * @param g Second equation.
 * @return Two candidate solutions as @c Eigen::Vector2d.
 */
template <typename FuncF, typename FuncG>
std::array<Eigen::Vector2d, 2> solve2D(FuncF&& f, FuncG&& g)
{
    return solve2D(std::forward<FuncF>(f), std::forward<FuncG>(g),
        DEFAULT_SPATIAL_GUESSES);
}

} // namespace Gcs::Equations

#endif // NEWTON_RAPHSON_HPP
