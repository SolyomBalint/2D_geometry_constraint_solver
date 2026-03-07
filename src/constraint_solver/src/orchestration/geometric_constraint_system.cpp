#include "orchestration/geometric_constraint_system.hpp"

// General STD/STL headers
#include <iostream>
#include <stdexcept>

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

} // namespace Gcs
