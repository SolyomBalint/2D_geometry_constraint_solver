#ifndef STREE_TOP_DOWN_STRATEGY_HPP
#define STREE_TOP_DOWN_STRATEGY_HPP

// General STD/STL headers
#include <vector>

// Custom headers
#include "model/gcs_data_structures.hpp"
#include "orchestration/solving_strategy.hpp"
#include <structures/binary_tree.hpp>

namespace Gcs {

class DeficitStreeBasedTopDownStrategy : public GcsSolvingStrategy {
public:
    Constrainedness checkConstraintGraphConstrainedness(
        const ConstraintGraph& gcs) override;

    bool resolve(ConstraintGraph& gcs) override;

    std::vector<ConstraintGraph> decomposeConstraintGraph(
        ConstraintGraph& gcs) override;

    void solveGcs(std::vector<ConstraintGraph>& splitComponents) override;

    MathUtils::BinaryTree<ConstraintGraph> getSTreeDecomposition(
        ConstraintGraph& gcs);

    ~DeficitStreeBasedTopDownStrategy() override = default;
};

} // namespace Gcs

#endif // STREE_TOP_DOWN_STRATEGY_HPP
