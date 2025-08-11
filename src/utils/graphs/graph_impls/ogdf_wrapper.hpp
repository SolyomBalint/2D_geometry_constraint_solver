/**
 * WARNING: This file is AI generated and is not tested. This wrapper is only used for rapid prototyping
 */

#ifndef OGDF_GRAPH_WRAPPER_HPP
#define OGDF_GRAPH_WRAPPER_HPP

#include "../graph_interface.hpp"
#include <exception>
#include <common/common_uuid.hpp>
#include <ogdf/basic/Graph.h>
#include <ogdf/basic/NodeArray.h>
#include <ogdf/basic/simple_graph_alg.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <ogdf/graphalg/Triconnectivity.h>
#include <ogdf/basic/extended_graph_alg.h>
#include <stack>
#include <vector>
#include <unordered_map>

namespace MathUtils {

const auto OGDF_GRAPH_LOGGER = spdlog::stdout_color_mt("OGDF_GRAPH");

class OGDFNodeImpl {
private:
    ogdf::node node_;

public:
    OGDFNodeImpl() = default;
    OGDFNodeImpl(ogdf::node n)
        : node_(n)
    {
    }

    int getId() const { return node_->index(); }
    ogdf::node getNode() const { return node_; }
};

class OGDFEdgeImpl {
private:
    ogdf::edge edge_;

public:
    OGDFEdgeImpl(ogdf::edge edge)
        : edge_(edge)
    {
    }

    int getId() const { return edge_->index(); }
    ogdf::edge getEdge() const { return edge_; }
};

template <typename NodeStoredObject, typename EdgeStoredObject> class OGDFGraphImpl {
public:
    using NodeType = NodeInterface<OGDFNodeImpl, NodeStoredObject>;
    using EdgeType = EdgeInterface<OGDFEdgeImpl, EdgeStoredObject>;

private:
    ogdf::Graph graph_;
    std::unordered_map<int, NodeType> nodeMap_;
    std::unordered_map<int, EdgeType> edgeMap_;

public:
    NodeType& addNode(std::shared_ptr<NodeStoredObject> obj)
    {
        ogdf::node newNode = graph_.newNode();
        auto newNodeId = newNode->index();

        nodeMap_.emplace(newNodeId, NodeType { OGDFNodeImpl { newNode }, obj });
        return nodeMap_.at(newNodeId);
    }

    EdgeType& addEdge(const NodeType& node1, const NodeType& node2, std::shared_ptr<EdgeStoredObject> obj)
    {
        auto it1 = nodeMap_.find(node1.getId());
        auto it2 = nodeMap_.find(node2.getId());

        if (it1 != nodeMap_.end() && it2 != nodeMap_.end()) {
            ogdf::edge newEdge = graph_.newEdge(it1->second.getImpl().getNode(), it2->second.getImpl().getNode());
            auto newEdgeId = newEdge->index();

            edgeMap_.emplace(newEdgeId, EdgeType { OGDFEdgeImpl { newEdge }, obj });
            return edgeMap_.at(newEdgeId);
        }

        throw std::runtime_error("Invalid nodes for edge creation");
    }

    std::vector<NodeType> getCutVertices() const
    {
        ogdf::ArrayBuffer<ogdf::node> cutVertices;
        ogdf::findCutVertices(graph_, cutVertices);

        std::vector<NodeType> outNodes;

        for (auto& node : cutVertices) {
            outNodes.push_back(nodeMap_.at(node->index()));
        }

        return outNodes;
    }

    void getSeparationPairs(NodeType* outNodeOne, NodeType* outNodeTwo) const
    {
        ogdf::node separatorNodeOne;
        ogdf::node separatorNodeTwo;
        bool isTriconnected = false;

        ogdf::Triconnectivity(graph_, isTriconnected, separatorNodeOne, separatorNodeTwo);

        if (isTriconnected) {
            throw std::logic_error("Graph is triconnected, no separation pairs exist");
        }

        if (separatorNodeTwo == nullptr) {
            throw std::logic_error("Expected separation pair but got nullptr");
        }

        // Debug: Check if keys exist in map
        int index1 = separatorNodeOne->index();
        int index2 = separatorNodeTwo->index();

        std::cout << "Looking for indices: " << index1 << ", " << index2 << std::endl;
        std::cout << "nodeMap_ size: " << nodeMap_.size() << std::endl;

        auto it1 = nodeMap_.find(index1);
        auto it2 = nodeMap_.find(index2);

        if (it1 == nodeMap_.end()) {
            throw std::logic_error("Node index " + std::to_string(index1) + " not found in nodeMap_");
        }
        if (it2 == nodeMap_.end()) {
            throw std::logic_error("Node index " + std::to_string(index2) + " not found in nodeMap_");
        }

        *outNodeOne = it1->second;
        *outNodeTwo = it2->second;
    }

    std::vector<OGDFGraphImpl> separateByVerticesByDuplication(const std::vector<NodeType>& separatorNodes)
    {
        using node = ogdf::node;

        std::vector<ogdf::Graph> subGraphs {};

        auto numberOfNodes = graph_.nodes.size();
        std::vector<node> ogdfSepNodes {};

        for (auto n : separatorNodes) {
            ogdfSepNodes.push_back(n.getImpl().getNode());
        }

        std::vector<node> absoluteVisitedNodes { ogdfSepNodes };

        OGDF_GRAPH_LOGGER->debug(std::format("Before while loop, number of total nodes: {}", numberOfNodes));
        while (numberOfNodes != absoluteVisitedNodes.size()) {
            std::vector<node> currentlyVisitedNodes { ogdfSepNodes };
            std::stack<node> nodeStack {};

            // TODO: with large graphs this may be slow af
            auto nextNode = graph_.chooseNode([&absoluteVisitedNodes](node n) {
                return std::ranges::find(absoluteVisitedNodes, n) == absoluteVisitedNodes.end();
            });
            if (nextNode != nullptr) {
                nodeStack.push(nextNode);
            }

            OGDF_GRAPH_LOGGER->debug("Before dfs");
            while (!nodeStack.empty()) {
                auto currentNode = nodeStack.top();
                nodeStack.pop();

                // FIX 1: Check if node is already visited before adding
                bool alreadyInCurrently
                    = std::ranges::find(currentlyVisitedNodes, currentNode) != currentlyVisitedNodes.end();
                bool alreadyInAbsolute
                    = std::ranges::find(absoluteVisitedNodes, currentNode) != absoluteVisitedNodes.end();

                if (alreadyInCurrently || alreadyInAbsolute) {
                    continue; // Skip this node if already processed
                }

                currentlyVisitedNodes.push_back(currentNode);
                absoluteVisitedNodes.push_back(currentNode);

                OGDF_GRAPH_LOGGER->debug("Before dfs edge iteration");

                for (auto edge : currentNode->adjEntries) {
                    auto adjNode = edge->twinNode();

                    OGDF_GRAPH_LOGGER->debug(std::format("Current Node: {}", adjNode->index()));

                    // FIX 2: Check against both currentlyVisitedNodes AND absoluteVisitedNodes
                    bool foundInCurrently
                        = std::ranges::find(currentlyVisitedNodes, adjNode) != currentlyVisitedNodes.end();
                    bool foundInAbsolute
                        = std::ranges::find(absoluteVisitedNodes, adjNode) != absoluteVisitedNodes.end();

                    if (!foundInCurrently && !foundInAbsolute) {
                        OGDF_GRAPH_LOGGER->debug("after dfs node visit check");
                        nodeStack.push(adjNode); // FIX 3: Use adjNode consistently
                    }
                }
            }

            ogdf::Graph subGraph {};
            ogdf::NodeArray<node> originalToNewNodeMapping {};
            ogdf::EdgeArray<ogdf::edge> originalToNewEdgeMapping {};

            ogdf::List<node> tempList {};
            for (auto n : currentlyVisitedNodes) {
                tempList.pushBack(n);
            }
            ogdf::ListIterator<ogdf::node> it { tempList.begin() };
            ogdf::inducedSubGraph(graph_, it, subGraph, originalToNewNodeMapping, originalToNewEdgeMapping);

            ogdf::List<node> nodeInfoList {};
            subGraph.allNodes(nodeInfoList);
            for (auto n : nodeInfoList) {
                OGDF_GRAPH_LOGGER->debug(std::format("subgraph node index: {}", n->index()));
            }

            ogdf::List<ogdf::edge> edgeInfoList {};
            subGraph.allEdges(edgeInfoList);
            for (auto e : edgeInfoList) {
                OGDF_GRAPH_LOGGER->debug(
                    std::format("subgraph edge ends: {} - {}", e->source()->index(), e->target()->index()));
            }

            subGraphs.push_back(std::move(subGraph));
            OGDF_GRAPH_LOGGER->debug("subgraph created");
            OGDF_GRAPH_LOGGER->debug(std::format("number of total visited nodes: {}", absoluteVisitedNodes.size()));
        }

        // FIX 4: Don't forget to return the result!
        std::vector<OGDFGraphImpl> result;
        // TODO: Convert subGraphs to OGDFGraphImpl instances
        return result;
    }
};

static_assert(GraphImplRequirements<OGDFGraphImpl<int, int>, OGDFNodeImpl, int, OGDFEdgeImpl, int>);

template <typename NodeStoredObj, typename EdgeStoredObj>
using Graph = GraphInterface<OGDFGraphImpl<NodeStoredObj, EdgeStoredObj>, OGDFNodeImpl, NodeStoredObj, OGDFEdgeImpl,
    EdgeStoredObj>;

} // namespace MathUtils

#endif // OGDF_GRAPH_WRAPPER_HPP
