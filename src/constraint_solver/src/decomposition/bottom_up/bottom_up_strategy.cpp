#include <gcs/decomposition/bottom_up/bottom_up_strategy.hpp>

// General STD/STL headers
#include <stdexcept>

// Custom headers
#include "solving/bottom_up_plan_solver.hpp"

namespace Gcs {

Constrainedness BottomUpDrPlanStrategy::checkConstraintGraphConstrainedness(
    const ConstraintGraph& gcs)
{
    const int nodeCount = static_cast<int>(gcs.nodeCount());
    const int edgeCount = static_cast<int>(gcs.edgeCount());
    const int deficit = (2 * nodeCount - 3) - edgeCount;

    if (deficit < 0) {
        return Constrainedness::INCONSISTENTLY_OVER_CONSTRAINED;
    }

    if (deficit == 0) {
        return Constrainedness::WELL_CONSTRAINED;
    }

    return Constrainedness::UNDER_CONSTRAINED;
}

bool BottomUpDrPlanStrategy::resolve(ConstraintGraph& /*gcs*/)
{
    return false;
}

std::vector<ConstraintGraph> BottomUpDrPlanStrategy::decomposeConstraintGraph(
    ConstraintGraph& gcs)
{
    m_lastDecomposedGraph = &gcs;
    m_lastReductionResult = reduceBottomUp(gcs);

    return {};
}

void BottomUpDrPlanStrategy::solveGcs(
    std::vector<ConstraintGraph>& /*splitComponents*/)
{
    if (m_lastDecomposedGraph == nullptr
        || !m_lastReductionResult.has_value()) {
        throw std::runtime_error(
            "Bottom-up solve called without prior decomposition");
    }

    const auto solved = solveBottomUpPlans(
        m_lastReductionResult->rootPlans, *m_lastDecomposedGraph);
    if (solved.has_value()) {
        return;
    }

    switch (solved.error()) {
    case BottomUpPlanSolveError::InvalidPlanNode:
        throw std::runtime_error("Invalid bottom-up plan node in root plan");
    case BottomUpPlanSolveError::PrimitiveSolveFailed:
        throw std::runtime_error("Failed to solve bottom-up primitive node");
    case BottomUpPlanSolveError::MergeSolveFailed:
        throw std::runtime_error("Failed to solve bottom-up Merge3 node");
    case BottomUpPlanSolveError::WriteBackFailed:
        throw std::runtime_error("Failed to write bottom-up cluster pose");
    }

    throw std::runtime_error("Unknown bottom-up plan solver error");
}

} // namespace Gcs
