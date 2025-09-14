#include "geometric_constraint_system.hpp"
#include <stdexcept>
#include <queue>

namespace Gcs {

bool defaultDetectorFunc(
    const MathUtils::Graph<Element, Constraint>& constraintGraph)
{
    // Based on Lamans theorem, only works for constraint graph with distance
    // constraints and point elements
    return constraintGraph.getEdgeCount()
        == 2 * constraintGraph.getNodeCount() - 3;
}

void defaultResolverFunc(MathUtils::Graph<Element, Constraint>& constraintGraph)
{
    throw std::runtime_error("defaultResolverFunc not implemented");
}

std::vector<MathUtils::Graph<Element, Constraint>> defaultDecompositorFunc(
    MathUtils::Graph<Element, Constraint>& constraintGraph)
{
    // Checking if graph is biconnected, solving it is currently out of scope
    if (!constraintGraph.getCutVertices().empty()) {
        throw std::runtime_error(
            "Input graph is not biconnected. The algebraic approach requires "
            "biconnected graphs");
    }

    std::vector<MathUtils::Graph<Element, Constraint>> result {};

    // Use iterative decomposition without moving the original graph
    auto decompose = [&](MathUtils::Graph<Element, Constraint>& graph)
        -> std::vector<MathUtils::Graph<Element, Constraint>> {
        auto separationPair = graph.getSeparationPairs();

        if (separationPair.first.has_value()
            && separationPair.second.has_value()) {
            getGcsLogger()->debug(
                "separation pairs found in graph, separating...");
            return graph.separateByVerticesByDuplication(
                { separationPair.first.value(),
                    separationPair.second.value() });
        } else {
            getGcsLogger()->debug("No separation pairs found in graph");
            return {};
        }
    };

    // Process the original graph
    auto initialSubGraphs = decompose(constraintGraph);

    if (initialSubGraphs.empty()) {
        // No separation pairs found, return the original graph
        result.emplace_back(std::move(constraintGraph));
        return result;
    }

    // Queue to process subgraphs
    std::queue<MathUtils::Graph<Element, Constraint>> graphQueue;
    for (auto& subGraph : initialSubGraphs) {
        graphQueue.push(std::move(subGraph));
    }

    int count = 0;
    while (!graphQueue.empty()) {
        getGcsLogger()->debug("Size of result: {}", result.size());
        getGcsLogger()->debug("Current decomposition iteration: {}", count++);

        auto currentGraph = std::move(graphQueue.front());
        graphQueue.pop();

        auto subGraphs = decompose(currentGraph);

        if (subGraphs.empty()) {
            // No more separation pairs, add to result
            getGcsLogger()->debug("Adding triconnected component to result");
            result.emplace_back(std::move(currentGraph));
        } else {
            // Add new subgraphs to queue for further processing
            for (auto& subGraph : subGraphs) {
                getGcsLogger()->debug(
                    "Adding subgraph to queue for further decomposition");
                graphQueue.push(std::move(subGraph));
            }
        }
    }

    return result;
}

void defaultSolverFunc(
    std::vector<MathUtils::Graph<Element, Constraint>>& subgraphs)
{
    throw std::runtime_error("defaultSolverFunc not implemented");
}

} // namespace Gcs
