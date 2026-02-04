#ifndef POINT_POINT_SOLVERS_HPP
#define POINT_POINT_SOLVERS_HPP

// Custom headers
#include "../equations/equation_primitives.hpp"
#include "../equations/newton_raphson.hpp"
#include "../gcs_data_structures.hpp"
#include "../solve_result.hpp"
#include "./heuristics.hpp"
#include "./subproblem_solver_concept.hpp"

namespace Gcs::Solvers {

/**
 * @brief Solver for: 3 unsolved Points + 3 DistanceConstraints.
 *
 * Handles the base triangle case where no points have been solved
 * yet. Fixes P1 at origin, places P2 on the X-axis at distance d12,
 * then solves for P3 using two distance equations.
 *
 * @par Configuration
 * - 3 nodes, all Points, all unsolved
 * - 3 edges, all DistanceConstraints, no virtual edges
 */
struct ZeroFixedPointsTriangleSolver {

    /**
     * @brief Check whether a component matches this solver.
     * @param component The triconnected component to inspect.
     * @return True if the component is 3 unsolved Points with
     *         3 DistanceConstraints and no virtual edges.
     */
    static bool matches(const ConstraintGraph& component);

    /**
     * @brief Solve the 3-point triangle.
     *
     * Fix P1 at origin, P2 on X-axis at distance d12. Build two
     * @c pointToPointDistance equations for P3 and solve via
     * Newton-Raphson. Disambiguate using canvas orientation.
     *
     * @param component The triconnected component to solve in-place.
     * @return SolveResult indicating success or failure.
     */
    static SolveResult solve(ConstraintGraph& component);
};

static_assert(SubproblemSolver<ZeroFixedPointsTriangleSolver>);

/**
 * @brief Solver for: 2 solved Points + 1 unsolved Point +
 *        2 DistanceConstraints (+ 1 virtual or real edge).
 *
 * Handles the case where a component shares two separator nodes
 * with an already-solved component. The two fixed points provide
 * known positions; two distance constraints to the free point
 * yield two equations in two unknowns.
 *
 * @par Configuration
 * - 3 nodes, all Points, exactly 2 solved and 1 unsolved
 * - At least 2 non-virtual DistanceConstraint edges connecting
 *   the unsolved point to the two solved points
 */
struct TwoFixedPointsDistanceSolver {

    /**
     * @brief Check whether a component matches this solver.
     * @param component The triconnected component to inspect.
     * @return True if the component has 2 solved Points, 1 unsolved
     *         Point, and at least 2 DistanceConstraints.
     */
    static bool matches(const ConstraintGraph& component);

    /**
     * @brief Solve for the free point given two fixed points.
     *
     * Build two @c pointToPointDistance equations from the fixed
     * points to the free point and solve via Newton-Raphson.
     * Disambiguate using canvas orientation.
     *
     * @param component The triconnected component to solve in-place.
     * @return SolveResult indicating success or failure.
     */
    static SolveResult solve(ConstraintGraph& component);
};

static_assert(SubproblemSolver<TwoFixedPointsDistanceSolver>);

} // namespace Gcs::Solvers

#endif // POINT_POINT_SOLVERS_HPP
