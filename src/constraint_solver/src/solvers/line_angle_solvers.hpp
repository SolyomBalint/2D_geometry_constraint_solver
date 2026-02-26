#ifndef LINE_ANGLE_SOLVERS_HPP
#define LINE_ANGLE_SOLVERS_HPP

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
 * @brief Solver for: 2 unsolved Lines + 1 unsolved Point
 *        + 1 AngleConstraint + 2 DistanceConstraints.
 *
 * Handles the base triangle case containing two lines and one
 * point, where nothing has been solved yet. The two lines are
 * connected by an angle constraint, and the point has distance
 * constraints to both lines.
 *
 * Anchoring strategy: fix line 1 on the X-axis through the
 * origin, then place the point at @c (0, signedDist) directly
 * above/below the origin. This consumes all three rigid-body
 * degrees of freedom (fixing a line removes direction + offset
 * = 2 DOF; pinning the point's along-line projection removes
 * the remaining translation DOF).
 *
 * Line 2's unit normal is then solved via Newton-Raphson using
 * the @c lineNormalAngleConstraint + @c unitNormalConstraint
 * system. Its perpendicular offset is computed from the distance
 * constraint between the point and line 2.
 *
 * @par Configuration
 * - 3 nodes: exactly 2 Lines, 1 Point, all unsolved
 * - 3 edges: exactly 1 AngleConstraint (between the two Lines)
 *   and 2 DistanceConstraints (Point to each Line)
 */
struct ZeroFixedLLPAngleTriangleSolver {

    /**
     * @brief Check whether a component matches this solver.
     * @param component The triconnected component to inspect.
     * @return True if the component has 2 unsolved Lines,
     *         1 unsolved Point, 1 AngleConstraint between Lines,
     *         and 2 DistanceConstraints from Point to Lines.
     */
    static bool matches(const ConstraintGraph& component);

    /**
     * @brief Solve the Line-Line-Point angle triangle.
     *
     * Fix line 1 on the X-axis, place the point at
     * @c (0, signedDistPointToLine1). Solve line 2's unit
     * normal from the angle constraint via Newton-Raphson,
     * then compute line 2's perpendicular offset from the
     * point-to-line-2 distance. Reconstruct all endpoints.
     *
     * @param component The triconnected component to solve.
     * @return SolveResult indicating success or failure.
     */
    static SolveResult solve(ConstraintGraph& component);
};

static_assert(SubproblemSolver<ZeroFixedLLPAngleTriangleSolver>);

// ================================================================
// Partially solved (separator nodes from previously solved
// components) configurations
// ================================================================

/**
 * @brief Solver for: 1 solved Line + 1 solved Point
 *        + 1 unsolved Line + 1 AngleConstraint
 *        + 1 DistanceConstraint.
 *
 * The solved line and point are separator nodes from a previously
 * solved component. The free line is determined by an angle
 * constraint to the fixed line and a distance constraint from
 * the fixed point.
 *
 * The free line's unit normal is solved via Newton-Raphson using
 * the @c lineNormalAngleConstraint + @c unitNormalConstraint
 * system. The perpendicular offset is computed from the
 * point-to-free-line distance.
 *
 * @par Configuration
 * - 3 nodes: 2 Lines (1 solved, 1 unsolved), 1 Point (solved)
 * - Non-virtual constraints: 1 AngleConstraint (between the
 *   two Lines) and 1 DistanceConstraint (Point to free Line)
 * - Virtual edge between the solved Line and solved Point
 */
struct FixedLineAndPointFreeLineSolver {

    /**
     * @brief Check whether a component matches this solver.
     * @param component The triconnected component to inspect.
     * @return True if 1 solved Line + 1 solved Point
     *         + 1 unsolved Line with matching constraints.
     */
    static bool matches(const ConstraintGraph& component);

    /**
     * @brief Solve for the free line given a fixed line and
     *        point.
     *
     * Parameterize the free line by its unit normal
     * @c (nx, ny), build the angle + unit-normal equation
     * system, solve via Newton-Raphson, compute the
     * perpendicular offset from the point distance, and
     * reconstruct endpoints.
     *
     * @param component The triconnected component to solve.
     * @return SolveResult indicating success or failure.
     */
    static SolveResult solve(ConstraintGraph& component);
};

static_assert(SubproblemSolver<FixedLineAndPointFreeLineSolver>);

} // namespace Gcs::Solvers

#endif // LINE_ANGLE_SOLVERS_HPP
