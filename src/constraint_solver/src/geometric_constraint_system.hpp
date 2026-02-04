#ifndef GEOMETRIC_CONSTRAINT_SYSTEM_HPP
#define GEOMETRIC_CONSTRAINT_SYSTEM_HPP

#include "gcs_data_structures.hpp"
#include <memory>
#include <structures/binary_tree.hpp>
#include <utility>
#include <vector>

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

class GeometricConstraintSystem final {
public:
    void solveGeometricConstraintSystem(ConstraintGraph&);
    GeometricConstraintSystem(std::unique_ptr<GcsSolvingStrategy> strategy)
        : m_strategy(std::move(strategy))
    {
    }

private:
    std::unique_ptr<GcsSolvingStrategy> m_strategy;
};

}

#endif
