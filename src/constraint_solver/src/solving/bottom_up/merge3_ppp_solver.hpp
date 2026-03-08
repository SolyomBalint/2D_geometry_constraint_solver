#ifndef MERGE3_PPP_SOLVER_HPP
#define MERGE3_PPP_SOLVER_HPP

// Custom headers
#include "solving/bottom_up/merge3_case_solver.hpp"

namespace Gcs::Solvers::BottomUp {

struct Merge3PppSolver {
    [[nodiscard]] static std::optional<ClusterPose> solve(
        const Merge3Context& context);
};

static_assert(Merge3CaseSolver<Merge3PppSolver>);

} // namespace Gcs::Solvers::BottomUp

#endif // MERGE3_PPP_SOLVER_HPP
