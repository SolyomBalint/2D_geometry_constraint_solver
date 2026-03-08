#ifndef BOTTOM_UP_PLAN_SOLVER_HPP
#define BOTTOM_UP_PLAN_SOLVER_HPP

// General STD/STL headers
#include <expected>

// Custom headers
#include "decomposition/bottom_up/bottom_up_reducer.hpp"
#include "model/gcs_data_structures.hpp"

namespace Gcs {

enum class BottomUpPlanSolveError {
    InvalidPlanNode,
    PrimitiveSolveFailed,
    MergeSolveFailed,
    WriteBackFailed,
};

[[nodiscard]] std::expected<void, BottomUpPlanSolveError> solveBottomUpPlans(
    const std::vector<PlanTree>& rootPlans, ConstraintGraph& graph);

} // namespace Gcs

#endif // BOTTOM_UP_PLAN_SOLVER_HPP
