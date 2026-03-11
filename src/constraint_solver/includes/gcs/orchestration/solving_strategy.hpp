#ifndef GCS_ORCHESTRATION_SOLVING_STRATEGY_HPP
#define GCS_ORCHESTRATION_SOLVING_STRATEGY_HPP

// General STD/STL headers
#include <vector>

// Custom headers
#include <gcs/export.hpp>
#include <gcs/model/gcs_data_structures.hpp>

namespace Gcs {

enum class Constrainedness {
    UNDER_CONSTRAINED,
    WELL_CONSTRAINED,
    CONSISTENTLY_OVER_CONSTRAINED,
    INCONSISTENTLY_OVER_CONSTRAINED
};

class GCS_API GcsSolvingStrategy {
public:
    GcsSolvingStrategy() = default;
    GcsSolvingStrategy(const GcsSolvingStrategy&) = default;
    GcsSolvingStrategy(GcsSolvingStrategy&&) = default;
    GcsSolvingStrategy& operator=(const GcsSolvingStrategy&) = default;
    GcsSolvingStrategy& operator=(GcsSolvingStrategy&&) = default;

    virtual Constrainedness checkConstraintGraphConstrainedness(
        const ConstraintGraph&)
        = 0;

    virtual bool resolve(ConstraintGraph&) = 0;

    virtual std::vector<ConstraintGraph> decomposeConstraintGraph(
        ConstraintGraph&)
        = 0;

    virtual void solveGcs(std::vector<ConstraintGraph>&) = 0;

    virtual ~GcsSolvingStrategy() = default;
};

} // namespace Gcs

#endif // GCS_ORCHESTRATION_SOLVING_STRATEGY_HPP
