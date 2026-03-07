#include "decomposition/top_down/stree_top_down_strategy.hpp"

// General STD/STL headers
#include <ranges>

// Custom headers
#include "solving/component_solver.hpp"
#include <structures/separation_pairs.hpp>

namespace Gcs {

Constrainedness
DeficitStreeBasedTopDownStrategy::checkConstraintGraphConstrainedness(
    const ConstraintGraph& gcs)
{
    const auto& deficit = (2 * gcs.nodeCount() - 3) - gcs.edgeCount();
    Constrainedness out {};
    if (deficit < 0) {
        out = Constrainedness::INCONSISTENTLY_OVER_CONSTRAINED;
    } else if (deficit == 0) {
        out = Constrainedness::WELL_CONSTRAINED;
    } else if (deficit > 0) {
        out = Constrainedness::UNDER_CONSTRAINED;
    }

    return out;
}

bool DeficitStreeBasedTopDownStrategy::resolve(ConstraintGraph& /*gcs*/)
{
    return false;
}

std::vector<ConstraintGraph>
DeficitStreeBasedTopDownStrategy::decomposeConstraintGraph(ConstraintGraph& gcs)
{

    return getSTreeDecomposition(gcs).getLeafValuesPostOrder();
}

void DeficitStreeBasedTopDownStrategy::solveGcs(
    std::vector<ConstraintGraph>& splitComponents)
{
    std::ranges::for_each(splitComponents, classifyAndSolve);
}

MathUtils::BinaryTree<ConstraintGraph>
DeficitStreeBasedTopDownStrategy::getSTreeDecomposition(ConstraintGraph& gcs)
{
    using namespace MathUtils;

    auto analysis = [](this auto&& self,
                        ConstraintGraph& gcs) -> BinaryTree<ConstraintGraph> {
        if (isTriconnected(gcs.getGraph()).value()) {
            return BinaryTree<ConstraintGraph>::make(gcs, {}, {});
        }

        auto [g1Info, g2Info] = gcs.getSeparatingGraphs();

        if (g1Info.separtionGraph.getDeficit()
            > g2Info.separtionGraph.getDeficit()) {
            g1Info.separtionGraph.addVirtualEdge(
                g1Info.originalToNewSepNodeA.second,
                g1Info.originalToNewSepNodeB.second);
            return BinaryTree<ConstraintGraph>::make(
                gcs, self(g1Info.separtionGraph), self(g2Info.separtionGraph));
        }

        // Always put tree with without Virtual Edge to the right
        g2Info.separtionGraph.addVirtualEdge(
            g2Info.originalToNewSepNodeA.second,
            g2Info.originalToNewSepNodeB.second);

        return BinaryTree<ConstraintGraph>::make(
            gcs, self(g2Info.separtionGraph), self(g1Info.separtionGraph));
    };

    return analysis(gcs);
}

} // namespace Gcs
