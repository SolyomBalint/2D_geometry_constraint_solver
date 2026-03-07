#ifndef SOLVING_STRATEGY_HPP
#define SOLVING_STRATEGY_HPP

// General STD/STL headers
#include <vector>

// Custom headers
#include "model/gcs_data_structures.hpp"

namespace Gcs {

enum class Constrainedness {
    UNDER_CONSTRAINED,
    WELL_CONSTRAINED,
    CONSISTENTLY_OVER_CONSTRAINED,
    INCONSISTENTLY_OVER_CONSTRAINED
};

class GcsSolvingStrategy {
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

#endif // SOLVING_STRATEGY_HPP
