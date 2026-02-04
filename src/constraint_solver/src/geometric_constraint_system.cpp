#include "component_solver.hpp"
#include "gcs_data_structures.hpp"
#include "solve_result.hpp"
#include <algorithm>
#include <format>
#include <geometric_constraint_system.hpp>
#include <iostream>
#include <ranges>
#include <stdexcept>
#include <structures/binary_tree.hpp>
#include <structures/separation_pairs.hpp>
#include <vector>

namespace Gcs {

void GeometricConstraintSystem::solveGeometricConstraintSystem(
    ConstraintGraph& gcs)
{
    if (const auto& constrainedness
        = m_strategy->checkConstraintGraphConstrainedness(gcs);
        constrainedness != Constrainedness::WELL_CONSTRAINED) {

        if (!m_strategy->resolve(gcs)) {
            throw std::runtime_error("Gcs is not well-constrained, current "
                                     "algorithms do not support such inputs");
        }
    }

    std::cerr << "Solver Called\n";
    auto decomposition = m_strategy->decomposeConstraintGraph(gcs);

    m_strategy->solveGcs(decomposition);
}

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

}
