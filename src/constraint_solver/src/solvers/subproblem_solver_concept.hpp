#ifndef SUBPROBLEM_SOLVER_CONCEPT_HPP
#define SUBPROBLEM_SOLVER_CONCEPT_HPP

// General STD/STL headers
#include <concepts>

// Custom headers
#include "../gcs_data_structures.hpp"
#include "../solve_result.hpp"

namespace Gcs::Solvers {

/**
 * @brief Concept that every subproblem solver must satisfy.
 *
 * A SubproblemSolver is a stateless struct with two static methods:
 *
 * - @c matches: inspects a triconnected component and returns whether
 *   this solver knows how to handle it. The check examines node/edge
 *   counts, element types, constraint types, and solved status.
 *
 * - @c solve: assumes @c matches() returned true, solves the component
 *   in-place by building equations from primitives, running
 *   Newton-Raphson, disambiguating, and updating element positions.
 *
 * Every solver struct must pass a @c static_assert(SubproblemSolver<T>)
 * at its definition site to verify compliance at compile time.
 *
 * @tparam T The solver struct type.
 */
template <typename T>
concept SubproblemSolver = requires(
    ConstraintGraph& mutableComponent, const ConstraintGraph& constComponent) {
    { T::matches(constComponent) } -> std::same_as<bool>;
    { T::solve(mutableComponent) } -> std::same_as<SolveResult>;
};

} // namespace Gcs::Solvers

#endif // SUBPROBLEM_SOLVER_CONCEPT_HPP
