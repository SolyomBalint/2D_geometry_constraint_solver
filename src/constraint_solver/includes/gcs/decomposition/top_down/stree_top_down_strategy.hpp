#ifndef GCS_DECOMPOSITION_TOP_DOWN_STREE_TOP_DOWN_STRATEGY_HPP
#define GCS_DECOMPOSITION_TOP_DOWN_STREE_TOP_DOWN_STRATEGY_HPP

// General STD/STL headers
#include <vector>

// Custom headers
#include <gcs/export.hpp>
#include <gcs/model/gcs_data_structures.hpp>
#include <gcs/orchestration/solving_strategy.hpp>
#include <structures/binary_tree.hpp>

namespace Gcs {

class GCS_API DeficitStreeBasedTopDownStrategy : public GcsSolvingStrategy {
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

#endif // GCS_DECOMPOSITION_TOP_DOWN_STREE_TOP_DOWN_STRATEGY_HPP
