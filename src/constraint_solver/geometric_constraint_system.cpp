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
    constexpr int needNodesToCalculate = 2;

    //=================Point-Distance Functions=================

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

    //=================Point-Distance Functions=================

    //=================Points-On-Line Functions=================

    void solveFromTwoCalculatedNodesOneLine(ConstraintGraph& subGraph)
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

        if (!notSetElement.getStoredObj()->isElementType<Line>()) {
            throw std::runtime_error(
                "Trying to set node to line while it is not a line");
        }

        const auto& p1 = setElemnts.front().getStoredObj()->getElement<Point>();
        const auto& p2 = setElemnts.back().getStoredObj()->getElement<Point>();
        notSetElement.getStoredObj()->updateElementPosition(
            p1.x, p1.y, p2.x, p2.y);
    }

    void solveFromOneCalculatedNodesOneLine(ConstraintGraph& subGraph)
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

        Node fixedPointNode {};
        Node fixedLineNode {};

        if (setElemnts.front().getStoredObj()->isElementType<Point>()) {
            fixedPointNode = setElemnts.front();
            fixedLineNode = setElemnts.back();
        } else {
            fixedPointNode = setElemnts.back();
            fixedLineNode = setElemnts.front();
        }

        auto fixedLine = fixedLineNode.getStoredObj()->getElement<Line>();
        auto fixedPoint = fixedPointNode.getStoredObj()->getElement<Point>();
        auto unsetPoint = notSetElement.getStoredObj()->getElement<Point>();

        auto outputCoords = Solver::calculatePointToPointDistanceTriangleOnLine(
            { { fixedLine.x1, fixedLine.y1 }, { fixedLine.x2, fixedLine.y2 } },
            { fixedPoint.x, fixedPoint.y },
            subGraph.getEdgeBetweenNodes(fixedPointNode, notSetElement)
                ->getStoredObj()
                ->getConstraintValue(),
            { { fixedLine.originalX1OnCanvas, fixedLine.originalY1OnCanvas },
                { fixedLine.originalX2OnCanvas, fixedLine.originalY2OnCanvas },
                { fixedPoint.OiriginalXOnCanvas, fixedPoint.OriginalYOnCanvas },
                { unsetPoint.OiriginalXOnCanvas,
                    unsetPoint.OriginalYOnCanvas } });

        notSetElement.getStoredObj()->updateElementPosition(
            outputCoords.x, outputCoords.y);
    }

    void solveFromZeroCalculatedNodesOneLine(ConstraintGraph& subGraph)
    {
        using Node = ConstraintGraph::NodeType;
        auto nodes = subGraph.getNodes();

        std::vector<Node> pointNodes;

        Node lineNode;

        for (const auto& node : nodes) {
            if (node.getStoredObj()->isElementType<Point>()) {
                pointNodes.push_back(node);
            } else {
                lineNode = node;
            }
        }

        if (pointNodes.size() > 2) {
            throw std::runtime_error("No line in system, bad function call");
        }

        auto line = lineNode.getStoredObj()->getElement<Line>();
        pointNodes.front().getStoredObj()->updateElementPosition(0, 0);
        // Starting line from origin while keeping the original orinetation

        Solver::Coordinates2D linePoint2 { line.originalX2OnCanvas
                - line.originalX1OnCanvas,
            line.originalY2OnCanvas - line.originalY1OnCanvas };

        linePoint2.normalize();

        lineNode.getStoredObj()->updateElementPosition(
            0, 0, linePoint2.x, linePoint2.y);

        solveFromOneCalculatedNodesOneLine(subGraph);
    }
    //=================Points-On-Line Functions=================

    // TODO: // Overload pattern helper (C++20) would solve this nicely, later
    // refactor would help a lot
    void callSolverFunc(ConstraintGraph& graph, const int numberOfSolvedNodes)
    {
        const auto& nodes = graph.getNodes();
        const auto& edges = graph.getEdges();

        std::shared_ptr<Element> lineNode { nullptr };

        for (const auto& node : nodes) {
            if (node.getStoredObj()->isElementType<Line>()) {
                lineNode = node.getStoredObj();
            }
        }

        if (!lineNode && numberOfSolvedNodes == 0) {
            solveFromZeroCalculatedNodes(graph);
        } else if (!lineNode && numberOfSolvedNodes == 2) {
            solveFromTwoCalculatedNodes(graph);
        } else if (lineNode && numberOfSolvedNodes == 0) {
            solveFromZeroCalculatedNodesOneLine(graph);
        } else if (lineNode && numberOfSolvedNodes == 2) {
            if (lineNode->isElementSet()) {
                solveFromOneCalculatedNodesOneLine(graph);
            } else {
                solveFromTwoCalculatedNodesOneLine(graph);
            }
        }
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

// TODO take virtual edges into considerations
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
            auto virtualConstraint
                = std::make_shared<Constraint>(VirtualConstraint());
            return graph.separateByVerticesByDuplication(
                { separationPair.first.value(), separationPair.second.value() },
                virtualConstraint);
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

void debugPrint(const ConstraintGraph& graph)
{
    const auto& nodes = graph.getNodes();
    const auto& edges = graph.getEdges();

    for (const auto& node : nodes) {
        std::cerr << node.getStoredObj()->toString() << std::endl;
    }

    for (const auto& edge : edges) {
        std::cerr << edge.getStoredObj()->getConstraintName()
                  << " value: " << edge.getStoredObj()->getConstraintValue()
                  << std::endl;
    }
}

void defaultSolverFunc(std::vector<ConstraintGraph>& subgraphs)
{
    for (auto& graph : subgraphs) {
        if (graph.getNodeCount() != 3 || graph.getEdgeCount() != 3) {
            throw std::runtime_error(
                "Solver does not support non triangle subgraphs");
        }
    }

    auto noVirtualEdgeGraphs
        = subgraphs | std::views::filter([](const ConstraintGraph& graph) {
              return !graph.hasVirtualEdge();
          });

    std::cerr << "========================DEBUG===============" << std::endl;

    // Calling for first graph with 0 solved nodes
    std::cerr << "Calling first graph" << std::endl;
    callSolverFunc(noVirtualEdgeGraphs.front(), 0);
    debugPrint(noVirtualEdgeGraphs.front());

    // NOTE: this is ugly, a better solutions should be used once there is time
    std::cerr << "Calling solvable graphs" << std::endl;
    while (true) {
        auto solvableGraphs = noVirtualEdgeGraphs
            | std::views::filter([](const ConstraintGraph& graph) {
                  return getNumberOfCalculatedNode(graph)
                      == needNodesToCalculate;
              });

        // There are no more solvable graphs
        if (solvableGraphs.empty()) {
            break;
        }

        for (auto& graph : solvableGraphs) {
            callSolverFunc(graph, needNodesToCalculate);
            debugPrint(graph);
        }
    }

    std::cerr << "Calling virutal edges graphs" << std::endl;
    // Solving the virtual edge ones.
    auto virtualEdgeGraphs
        = subgraphs | std::views::filter([](const ConstraintGraph& graph) {
              return graph.hasVirtualEdge();
          });

    for (auto& graph : virtualEdgeGraphs) {
        callSolverFunc(graph, needNodesToCalculate);
        debugPrint(graph);
    }

    std::cerr << "========================DEBUG===============" << std::endl;
}

void GeometricConstraintSystem::solveGcsViaPipeline(
    ConstraintGraph& constraintGraph)
{
    while (!constraintnessDetector(constraintGraph)) {
        constraintnessResolver(constraintGraph);
    }

    auto subraphs = graphDecompositor(constraintGraph);
    subgraphSolver(subraphs);
}

} // namespace Gcs
