#ifndef GCS_DECOMPOSITION_BOTTOM_UP_BOTTOM_UP_STRATEGY_HPP
#define GCS_DECOMPOSITION_BOTTOM_UP_BOTTOM_UP_STRATEGY_HPP

// General STD/STL headers
#include <optional>
#include <vector>

// Custom headers
#include <gcs/decomposition/bottom_up/bottom_up_reducer.hpp>
#include <gcs/export.hpp>
#include <gcs/model/gcs_data_structures.hpp>
#include <gcs/orchestration/solving_strategy.hpp>

namespace Gcs {

/**
 * @brief Bottom-up DR-plan based solving strategy.
 *
 * This strategy currently keeps the legacy strategy interface unchanged.
 * Bottom-up decomposition artifacts are stored as strategy state and can be
 * queried after decomposition.
 */
class GCS_API BottomUpDrPlanStrategy : public GcsSolvingStrategy {
public:
    Constrainedness checkConstraintGraphConstrainedness(
        const ConstraintGraph& gcs) override;

    bool resolve(ConstraintGraph& gcs) override;

    std::vector<ConstraintGraph> decomposeConstraintGraph(
        ConstraintGraph& gcs) override;

    void solveGcs(std::vector<ConstraintGraph>& splitComponents) override;

    /**
     * @brief Return the last bottom-up decomposition result, if available.
     */
    [[nodiscard]] const std::optional<BottomUpReductionResult>&
    getLastReductionResult() const
    {
        return m_lastReductionResult;
    }

    /**
     * @brief Return whether a decomposition result is currently available.
     */
    [[nodiscard]] bool hasReductionResult() const
    {
        return m_lastReductionResult.has_value();
    }

    ~BottomUpDrPlanStrategy() override = default;

private:
    ConstraintGraph* m_lastDecomposedGraph { nullptr };
    std::optional<BottomUpReductionResult> m_lastReductionResult;
};

} // namespace Gcs

#endif // GCS_DECOMPOSITION_BOTTOM_UP_BOTTOM_UP_STRATEGY_HPP
