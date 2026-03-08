#ifndef MERGE3_LLP_SOLVER_HPP
#define MERGE3_LLP_SOLVER_HPP

// Custom headers
#include "solving/bottom_up/merge3_case_solver.hpp"

namespace Gcs::Solvers::BottomUp {

struct Merge3LlpSolver {
    [[nodiscard]] static std::optional<ClusterPose> solve(
        const Merge3Context& context);
};

static_assert(Merge3CaseSolver<Merge3LlpSolver>);

} // namespace Gcs::Solvers::BottomUp

#endif // MERGE3_LLP_SOLVER_HPP
