#ifndef POINT_LINE_SOLVERS_HPP
#define POINT_LINE_SOLVERS_HPP

// Custom headers
#include "../equations/equation_primitives.hpp"
#include "../equations/newton_raphson.hpp"
#include "../gcs_data_structures.hpp"
#include "../solve_result.hpp"
#include "./heuristics.hpp"
#include "./subproblem_solver_concept.hpp"

namespace Gcs::Solvers {

// ================================================================
// Fully unsolved (anchor triangle) configurations
// ================================================================

/**
 * @brief Solver for: 2 unsolved Points + 1 unsolved Line
 *        + 3 DistanceConstraints.
 *
 * Handles the base triangle case containing two points and one
 * line, where nothing has been solved yet. Fixes point 1 at the
 * origin and point 2 on the X-axis, then solves for the line's
 * unit normal via Newton-Raphson. Line endpoints are reconstructed
 * by projecting the canvas endpoints onto the solved infinite line.
 *
 * @par Configuration
 * - 3 nodes: exactly 2 Points, 1 Line, all unsolved
 * - 3 edges: all DistanceConstraints, no virtual edges
 * - 1 point-to-point distance, 2 point-to-line distances
 */
struct ZeroFixedPPLTriangleSolver {

    /**
     * @brief Check whether a component matches this solver.
     * @param component The triconnected component to inspect.
     * @return True if the component has 2 unsolved Points, 1
     *         unsolved Line, and 3 DistanceConstraints.
     */
    static bool matches(const ConstraintGraph& component);

    /**
     * @brief Solve the Point-Point-Line triangle.
     *
     * Fix point 1 at origin, point 2 on X-axis. Determine
     * signed distances from the canvas layout. Parameterize the
     * line by its unit normal @c (nx, ny) and solve the
     * @c lineNormalSignedDistanceDiff + @c unitNormalConstraint
     * system via Newton-Raphson. Reconstruct line endpoints by
     * projecting canvas endpoints onto the solved infinite line.
     *
     * @param component The triconnected component to solve.
     * @return SolveResult indicating success or failure.
     */
    static SolveResult solve(ConstraintGraph& component);
};

static_assert(SubproblemSolver<ZeroFixedPPLTriangleSolver>);

// ================================================================
// Partially solved (separator nodes from previously solved
// components) configurations
// ================================================================

/**
 * @brief Solver for: 2 solved Points + 1 unsolved Line
 *        + DistanceConstraints.
 *
 * The two solved points are separator nodes from a previously
 * solved component. The free line is determined by two
 * point-to-line distance constraints.
 *
 * @par Configuration
 * - 3 nodes: 2 Points (solved), 1 Line (unsolved)
 * - At least 2 non-virtual DistanceConstraint edges from points
 *   to line; virtual edge between the two points
 */
struct TwoFixedPointsLineSolver {

    /**
     * @brief Check whether a component matches this solver.
     * @param component The triconnected component to inspect.
     * @return True if 2 solved Points + 1 unsolved Line with
     *         distance constraints.
     */
    static bool matches(const ConstraintGraph& component);

    /**
     * @brief Solve for the free line given two fixed points.
     *
     * Parameterize the line by its unit normal @c (nx, ny),
     * determine signed distances from the canvas layout, solve
     * via Newton-Raphson, and reconstruct endpoints.
     *
     * @param component The triconnected component to solve.
     * @return SolveResult indicating success or failure.
     */
    static SolveResult solve(ConstraintGraph& component);
};

static_assert(SubproblemSolver<TwoFixedPointsLineSolver>);

/**
 * @brief Solver for: 1 solved Point + 1 solved Line
 *        + 1 unsolved Point + DistanceConstraints.
 *
 * One point and the line are separator nodes from a previously
 * solved component. The free point is determined by a
 * point-to-point distance and a point-to-line distance.
 *
 * @par Configuration
 * - 3 nodes: 2 Points (1 solved, 1 unsolved), 1 Line (solved)
 * - At least 2 non-virtual DistanceConstraint edges connecting
 *   the free point to the solved elements; virtual edge between
 *   the solved point and the solved line
 */
struct FixedPointAndLineFreePointSolver {

    /**
     * @brief Check whether a component matches this solver.
     * @param component The triconnected component to inspect.
     * @return True if 1 solved Point + 1 solved Line + 1 unsolved
     *         Point with distance constraints.
     */
    static bool matches(const ConstraintGraph& component);

    /**
     * @brief Solve for the free point given a fixed point and line.
     *
     * Build a @c pointToPointDistance equation (from the fixed
     * point) and a @c pointToLineDistance equation (from the
     * fixed line), determine the signed distance from canvas
     * layout, and solve via Newton-Raphson.
     *
     * @param component The triconnected component to solve.
     * @return SolveResult indicating success or failure.
     */
    static SolveResult solve(ConstraintGraph& component);
};

static_assert(SubproblemSolver<FixedPointAndLineFreePointSolver>);

/**
 * @brief Solver for: 2 solved Lines + 1 unsolved Point
 *        + DistanceConstraints.
 *
 * Both lines are separator nodes from a previously solved
 * component. The free point is determined by two point-to-line
 * distance constraints.
 *
 * @par Configuration
 * - 3 nodes: 1 Point (unsolved), 2 Lines (solved)
 * - 2 non-virtual DistanceConstraint edges from point to each
 *   line; virtual edge between the two lines
 */
struct TwoFixedLinesFreePointSolver {

    /**
     * @brief Check whether a component matches this solver.
     * @param component The triconnected component to inspect.
     * @return True if 2 solved Lines + 1 unsolved Point with
     *         distance constraints.
     */
    static bool matches(const ConstraintGraph& component);

    /**
     * @brief Solve for the free point given two fixed lines.
     *
     * Build two @c pointToLineDistance equations (one per line),
     * determine signed distances from the canvas layout, and
     * solve via Newton-Raphson.
     *
     * @param component The triconnected component to solve.
     * @return SolveResult indicating success or failure.
     */
    static SolveResult solve(ConstraintGraph& component);
};

static_assert(SubproblemSolver<TwoFixedLinesFreePointSolver>);

} // namespace Gcs::Solvers

#endif // POINT_LINE_SOLVERS_HPP
