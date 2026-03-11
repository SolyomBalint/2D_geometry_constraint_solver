#ifndef GCS_ORCHESTRATION_GEOMETRIC_CONSTRAINT_SYSTEM_HPP
#define GCS_ORCHESTRATION_GEOMETRIC_CONSTRAINT_SYSTEM_HPP

// General STD/STL headers
#include <memory>

// Custom headers
#include <gcs/decomposition/top_down/stree_top_down_strategy.hpp>
#include <gcs/export.hpp>
#include <gcs/orchestration/solving_strategy.hpp>

namespace Gcs {

class GCS_API GeometricConstraintSystem final {
public:
    void solveGeometricConstraintSystem(ConstraintGraph&);
    GeometricConstraintSystem(std::unique_ptr<GcsSolvingStrategy> strategy)
        : m_strategy(std::move(strategy))
    {
    }

    [[nodiscard]] const GcsSolvingStrategy& getStrategy() const
    {
        return *m_strategy;
    }

    [[nodiscard]] GcsSolvingStrategy& getStrategy() { return *m_strategy; }

private:
    std::unique_ptr<GcsSolvingStrategy> m_strategy;
};

} // namespace Gcs

#endif // GCS_ORCHESTRATION_GEOMETRIC_CONSTRAINT_SYSTEM_HPP
