#ifndef GEOMETRIC_CONSTRAINT_SYSTEM_HPP
#define GEOMETRIC_CONSTRAINT_SYSTEM_HPP

#include "../utils/graphs/graph_impls/ogdf_wrapper.hpp"
#include "elements.hpp"
#include "constraints.hpp"
#include <vector>

namespace Gcs {

inline std::shared_ptr<spdlog::logger> getGcsLogger()
{
    static auto gcsLogger = spdlog::stdout_color_mt("GCS_LOGGER");
    return gcsLogger;
}

using DetectorFunc = bool (*)(const MathUtils::Graph<Element, Constraint>&);
using ResolverFunc = void (*)(MathUtils::Graph<Element, Constraint>&);
using DecompositorFunc = std::vector<MathUtils::Graph<Element, Constraint>> (*)(
    MathUtils::Graph<Element, Constraint>&);
using SolverFunc
    = void (*)(std::vector<MathUtils::Graph<Element, Constraint>>&);

/**
 * @brief Uses Lamans theorem for detecting under- or over-constrained graphs
 * @param The constraint graph that should be checked
 * @return bool: true if the graph is well-constrained, false otherwise
 */
bool defaultDetectorFunc(const MathUtils::Graph<Element, Constraint>&);

void defaultResolverFunc(MathUtils::Graph<Element, Constraint>&);

std::vector<MathUtils::Graph<Element, Constraint>> defaultDecompositorFunc(
    MathUtils::Graph<Element, Constraint>&);

void defaultSolverFunc(std::vector<MathUtils::Graph<Element, Constraint>>&);

class GeometricConstraintSystem {
private:
    MathUtils::Graph<Element, Constraint> constraintGraph {};
    DetectorFunc constraintnessDetector = defaultDetectorFunc;
    ResolverFunc constraintnessResolver = defaultResolverFunc;
    DecompositorFunc graphDecompositor = defaultDecompositorFunc;
    SolverFunc subgraphSolver = defaultSolverFunc;

public:
    GeometricConstraintSystem() = default;
    GeometricConstraintSystem(
        MathUtils::Graph<Element, Constraint>&& constraintGraph)
        : constraintGraph { std::move(constraintGraph) }
    {
    }

    std::vector<MathUtils::Graph<Element, Constraint>> testIngThisShit()
    {
        auto [sep1, sep2] = constraintGraph.getSeparationPairs();
        auto vec = constraintGraph.separateByVerticesByDuplication(
            { sep1.value(), sep2.value() });

        std::cout << vec.size() << std::endl;
        std::cout << vec[0].getNodeCount() << std::endl;
        std::cout << vec[1].getNodeCount() << std::endl;
        std::cout << vec[0].getEdgeCount() << std::endl;
        std::cout << vec[1].getEdgeCount() << std::endl;
        std::cout << constraintGraph.getNodeCount() << std::endl;
        std::cout << constraintGraph.getEdgeCount() << std::endl;

        // return graphDecompositor(constraintGraph);
        return vec;
    }

    GeometricConstraintSystem(const GeometricConstraintSystem&) = delete;
    GeometricConstraintSystem& operator=(const GeometricConstraintSystem&)
        = delete;
    GeometricConstraintSystem(GeometricConstraintSystem&&) = delete;
    GeometricConstraintSystem& operator=(GeometricConstraintSystem&&) = delete;

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
