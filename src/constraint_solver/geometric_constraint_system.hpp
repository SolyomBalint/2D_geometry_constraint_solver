#ifndef GEOMETRIC_CONSTRAINT_SYSTEM_HPP
#define GEOMETRIC_CONSTRAINT_SYSTEM_HPP

#include "../utils/graphs/graph_impls/ogdf_wrapper.hpp"
#include "elements.hpp"
#include "constraints.hpp"
#include <vector>

namespace Gcs {

using ConstraintGraph = MathUtils::Graph<Element, Constraint>;

inline std::shared_ptr<spdlog::logger> getGcsLogger()
{
    static auto gcsLogger = spdlog::stdout_color_mt("GCS_LOGGER");
    return gcsLogger;
}

using DetectorFunc = bool (*)(const ConstraintGraph&);
using ResolverFunc = void (*)(ConstraintGraph&);
using DecompositorFunc = std::vector<ConstraintGraph> (*)(ConstraintGraph&);
using SolverFunc = void (*)(std::vector<ConstraintGraph>&);

/**
 * @brief Uses Lamans theorem for detecting under- or over-constrained graphs
 * @param The constraint graph that should be checked
 * @return bool: true if the graph is well-constrained, false otherwise
 */
bool defaultDetectorFunc(const ConstraintGraph&);

void defaultResolverFunc(ConstraintGraph&);

std::vector<ConstraintGraph> defaultDecompositorFunc(ConstraintGraph&);

void defaultSolverFunc(std::vector<ConstraintGraph>&);

class GeometricConstraintSystem final {
private:
    DetectorFunc constraintnessDetector = defaultDetectorFunc;
    ResolverFunc constraintnessResolver = defaultResolverFunc;
    DecompositorFunc graphDecompositor = defaultDecompositorFunc;
    SolverFunc subgraphSolver = defaultSolverFunc;

public:
    GeometricConstraintSystem() = default;

    GeometricConstraintSystem(const GeometricConstraintSystem&) = delete;
    GeometricConstraintSystem& operator=(const GeometricConstraintSystem&)
        = delete;
    GeometricConstraintSystem(GeometricConstraintSystem&&) = delete;
    GeometricConstraintSystem& operator=(GeometricConstraintSystem&&) = delete;

    // NOTE: this is a temporary solution due to the botched move semantics of
    // OGDF
    void solveGcsViaPipeline(ConstraintGraph&);

    /**
     * @brief Sets the function used to detect whether the input graph is
     * well-constrained
     * @param detector Function pointer that takes a constraint graph and
     * returns true if the graph is well-constrained
     */
    void setConstraintnessDetector(DetectorFunc detector)
    {
        constraintnessDetector = detector;
    }

    /**
     * @brief Sets the function used to convert the graph to a well-constrained
     * one
     * @param resolver Function pointer that modifies a constraint graph to make
     * it well-constrained
     */
    void setConstraintnessResolver(ResolverFunc resolver)
    {
        constraintnessResolver = resolver;
    }

    /**
     * @brief Sets the function used to decompose the constraint graph into
     * subgraphs
     * @param decompositor Function pointer that takes a constraint graph and
     * returns a vector of subgraphs
     */
    void setGraphDecompositor(DecompositorFunc decompositor)
    {
        graphDecompositor = decompositor;
    }

    /**
     * @brief Sets the function used to solve all subgraphs and reconstruct the
     * original one
     * @param solver Function pointer that takes a vector of constraint
     * subgraphs, solves them, and reconstructs the original
     */
    void setSubgraphSolver(SolverFunc solver) { subgraphSolver = solver; }
};
} // namespace Gcs

#endif // GEOMETRIC_CONSTRAINT_SYSTEM_HPP
