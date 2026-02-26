#ifndef COMPONENT_SOLVER_HPP
#define COMPONENT_SOLVER_HPP

// Custom headers
#include "./gcs_data_structures.hpp"
#include "./solve_result.hpp"
#include "./solvers/point_line_solvers.hpp"
#include "./solvers/point_point_solvers.hpp"

namespace Gcs {

/**
 * @brief Classify a triconnected component and dispatch to the
 *        matching subproblem solver.
 *
 * Each solver's @c matches() is tried in order. The first match wins.
 * Returns @c SolveStatus::Unsupported if no solver matches — this is
 * the default error handling for unimplemented configurations.
 *
 * @par Extension
 * To add a new solver:
 * 1. Create a new struct satisfying @c SubproblemSolver in the
 *    appropriate @c *_solvers.hpp file.
 * 2. Add one @c if line here.
 * 3. Include the header above.
 *
 * @param component The triconnected component to solve.
 * @return SolveResult indicating the outcome.
 */
inline SolveResult classifyAndSolve(ConstraintGraph& component)
{
    // --- Fully unsolved configurations ---

    if (Solvers::ZeroFixedPointsTriangleSolver::matches(component))
        return Solvers::ZeroFixedPointsTriangleSolver::solve(component);

    if (Solvers::ZeroFixedPPLTriangleSolver::matches(component))
        return Solvers::ZeroFixedPPLTriangleSolver::solve(component);

    // --- Partially solved configurations (separator nodes from
    //     previously solved components) ---

    if (Solvers::TwoFixedPointsDistanceSolver::matches(component))
        return Solvers::TwoFixedPointsDistanceSolver::solve(component);

    if (Solvers::TwoFixedPointsLineSolver::matches(component))
        return Solvers::TwoFixedPointsLineSolver::solve(component);

    if (Solvers::FixedPointAndLineFreePointSolver::matches(component))
        return Solvers::FixedPointAndLineFreePointSolver::solve(component);

    if (Solvers::TwoFixedLinesFreePointSolver::matches(component))
        return Solvers::TwoFixedLinesFreePointSolver::solve(component);

    // --- Default: no solver found ---

    return SolveResult::unsupported(
        "No solver matches this component configuration");
}

} // namespace Gcs

#endif // COMPONENT_SOLVER_HPP
