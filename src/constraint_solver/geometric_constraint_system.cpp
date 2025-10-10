/**
 * @file
 * @brief
 * TODOS:  Currently there is no check for virtual edges anywhere, and the 3.
 * condition of the algebraic approahc isnot checked
 */

#include "geometric_constraint_system.hpp"
#include "constraint_equation_solver.hpp"
#include <stdexcept>
#include <queue>

namespace Gcs {

// TODO This whole nameless namespace should be refactored, most of to function
// are hard coded instead of real logic
namespace {

    constexpr int point1Index = 0;
    constexpr int point2Index = 1;
    constexpr int point3Index = 2;

    int getNumberOfCalculatedNode(const ConstraintGraph& subGraph)
    {
        auto nodes = subGraph.getNodes();

        int calculated = 0;
        for (const auto& node : nodes) {
            if (node.getStoredObj().get()->isElementSet()) {
                ++calculated;
            }
        }

        return calculated;
    }

    void solveFromZeroCalculatedNodes(ConstraintGraph& subGraph)
    {
        auto nodes = subGraph.getNodes();

        // for (const auto& node : nodes) {
        //     getGcsLogger()->debug(node.getStoredObj()->toString());
        // }
        //
        // for (const auto& edge : subGraph.getEdges()) {
        //     getGcsLogger()->debug(edge.getStoredObj()->getConstraintValue());
        // }

        auto node1 = nodes.at(point1Index);
        auto node2 = nodes.at(point2Index);
        auto node3 = nodes.at(point3Index);

        Point p1 = node1.getStoredObj()->getElement<Point>();
        Point p2 = node2.getStoredObj()->getElement<Point>();
        Point p3 = node3.getStoredObj()->getElement<Point>();

        const auto xToYDistance = subGraph.getEdgeBetweenNodes(node1, node2)
                                      .value()
                                      .getStoredObj()
                                      ->getConstraintValue();
        const auto xToZDistance = subGraph.getEdgeBetweenNodes(node1, node3)
                                      .value()
                                      .getStoredObj()
                                      ->getConstraintValue();
        const auto yToZDistance = subGraph.getEdgeBetweenNodes(node2, node3)
                                      .value()
                                      .getStoredObj()
                                      ->getConstraintValue();

        const auto [p1Coords, p2Coords, p3Coords]
            = Solver::calculatePointToPointDistanceTriangle(xToYDistance,
                xToZDistance, yToZDistance,
                { { p1.OiriginalXOnCanvas, p1.OriginalYOnCanvas },
                    { p2.OiriginalXOnCanvas, p2.OriginalYOnCanvas },
                    { p3.OiriginalXOnCanvas, p3.OriginalYOnCanvas } });

        node1.getStoredObj()->updateElementPosition(p1Coords.x, p1Coords.y);
        node2.getStoredObj()->updateElementPosition(p2Coords.x, p2Coords.y);
        node3.getStoredObj()->updateElementPosition(p3Coords.x, p3Coords.y);
    }

    void solveFromTwoCalculatedNodes(ConstraintGraph& subGraph)
    {

        using Node = ConstraintGraph::NodeType;
        auto nodes = subGraph.getNodes();

        std::vector<Node> setElemnts;

        Node notSetElement;

        for (const auto& node : nodes) {
            if (node.getStoredObj()->isElementSet()) {
                setElemnts.push_back(node);
            } else {
                notSetElement = node;
            }
        }
        if (setElemnts.size() > 2) {
            throw std::runtime_error(
                "Trying to solve triangle with more than two solved elements");
        }

        // for (const auto& node : setElemnts) {
        //     getGcsLogger()->debug(node.getStoredObj()->toString());
        // }
        //
        // for (const auto& edge : subGraph.getEdges()) {
        //     getGcsLogger()->debug(edge.getStoredObj()->getConstraintValue());
        // }
        // debug

        auto p1ToP3Distance
            = subGraph.getEdgeBetweenNodes(setElemnts.front(), notSetElement)
                  ->getStoredObj()
                  ->getConstraintValue();
        // fails in 116
        auto p2ToP3Distance
            = subGraph.getEdgeBetweenNodes(setElemnts.back(), notSetElement)
                  ->getStoredObj()
                  ->getConstraintValue();

        Point p1 { setElemnts.front().getStoredObj()->getElement<Point>() };
        Point p2 { setElemnts.back().getStoredObj()->getElement<Point>() };
        Point p3 { notSetElement.getStoredObj()->getElement<Point>() };

        auto outPutCoords
            = Solver::calculatePointToPointDistanceTriangleFromTwoFixedPoints(
                { p1.x, p1.y }, { p2.x, p2.y }, p2ToP3Distance, p1ToP3Distance,
                { { p1.OiriginalXOnCanvas, p1.OriginalYOnCanvas },
                    { p2.OiriginalXOnCanvas, p2.OriginalYOnCanvas },
                    { p3.OiriginalXOnCanvas, p3.OriginalYOnCanvas } });

        notSetElement.getStoredObj()->updateElementPosition(
            outPutCoords.x, outPutCoords.y);
    }
}

bool defaultDetectorFunc(const ConstraintGraph& constraintGraph)
{
    // Based on Lamans theorem, only works for constraint graph with distance
    // constraints and point elements
    return constraintGraph.getEdgeCount()
        == 2 * constraintGraph.getNodeCount() - 3;
}

void defaultResolverFunc(ConstraintGraph& constraintGraph)
{
    throw std::runtime_error("defaultResolverFunc not implemented");
}

std::vector<ConstraintGraph> defaultDecompositorFunc(
    ConstraintGraph& constraintGraph)
{
    // Checking if graph is biconnected, solving it is currently out of scope
    if (!constraintGraph.getCutVertices().empty()) {
        throw std::runtime_error(
            "Input graph is not biconnected. The algebraic approach requires "
            "biconnected graphs");
    }

    std::vector<ConstraintGraph> result {};

    // Use iterative decomposition without moving the original graph
    auto decompose
        = [&](ConstraintGraph& graph) -> std::vector<ConstraintGraph> {
        auto separationPair = graph.getSeparationPairs();

        if (separationPair.first.has_value()
            && separationPair.second.has_value()) {
            // getGcsLogger()->debug(
            //     "separation pairs found in graph, separating...");
            return graph.separateByVerticesByDuplication(
                { separationPair.first.value(),
                    separationPair.second.value() });
        } else {
            // getGcsLogger()->debug("No separation pairs found in graph");
            return {};
        }
    };

    // Process the original graph
    auto initialSubGraphs = decompose(constraintGraph);

    if (initialSubGraphs.empty()) {
        // No separation pairs found, return copy of the original graph
        result.emplace_back(constraintGraph);
        return result;
    }

    // Queue to process subgraphs
    std::queue<ConstraintGraph> graphQueue;
    for (const auto& subGraph : initialSubGraphs) {
        graphQueue.push(subGraph);
    }

    int count = 0;
    while (!graphQueue.empty()) {
        getGcsLogger()->debug("Size of result: {}", result.size());
        getGcsLogger()->debug("Current decomposition iteration: {}", count++);

        auto currentGraph = graphQueue.front();
        graphQueue.pop();

        auto subGraphs = decompose(currentGraph);

        if (subGraphs.empty()) {
            // No more separation pairs, add to result
            getGcsLogger()->debug("Adding triconnected component to result");
            result.emplace_back(currentGraph);
        } else {
            // Add new subgraphs to queue for further processing
            for (const auto& subGraph : subGraphs) {
                getGcsLogger()->debug(
                    "Adding subgraph to queue for further decomposition");
                graphQueue.push(subGraph);
            }
        }
    }

    return result;
}

void defaultSolverFunc(std::vector<ConstraintGraph>& subgraphs)
{
    for (auto& graph : subgraphs) {
        if (graph.getNodeCount() != 3 || graph.getEdgeCount() != 3) {
            throw std::runtime_error(
                "Solver does not support non triangle subgraphs");
        }
    }

    auto starterGraph = subgraphs.front();
    solveFromZeroCalculatedNodes(starterGraph);

    // NOTE: this is ugly, a better solutions should be used once there is time
    while (true) {
        auto solvableGraphs
            = subgraphs | std::views::filter([](const ConstraintGraph& graph) {
                  return getNumberOfCalculatedNode(graph) == 2;
              });

        // There are no more solvable graphs
        if (solvableGraphs.empty()) {
            break;
        }

        for (auto& graph : solvableGraphs) {
            solveFromTwoCalculatedNodes(graph);
        }
    }
}

void GeometricConstraintSystem::solveGcsViaPipeline(
    ConstraintGraph& constraintGraph)
{
    while (!constraintnessDetector(constraintGraph)) {
        constraintnessResolver(constraintGraph);
    }

    auto subraphs = graphDecompositor(constraintGraph);
    getGcsLogger()->debug("============================================");
    // getGcsLogger()->debug("============================================");
    // getGcsLogger()->debug("Number of found subgraphs: {}", subraphs.size());
    // int count { 0 };
    // for (const auto& graph : subraphs) {
    //     getGcsLogger()->debug("subgraphs: {}", count++);
    //     const auto& nodes = graph.getNodes();
    //     for (const auto& node : graph.getNodes()) {
    //         std::cerr << node.getStoredObj().get() << std::endl;
    //     }
    //
    //     getGcsLogger()->debug("getting edges between nodes");
    //
    //     for (const auto& edge : graph.getEdges()) {
    //         std::cerr << edge.getStoredObj().get() << std::endl;
    //     }
    // }

    getGcsLogger()->debug("============================================");
    subgraphSolver(subraphs);
}

} // namespace Gcs
