/**
 * WARNING: This file is AI generated and is not tested. This wrapper is only
 * used for rapid prototyping
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

inline std::shared_ptr<spdlog::logger> getOGDFGraphLogger()
{
    static auto logger = spdlog::stdout_color_mt("OGDF_GRAPH");
    return logger;
}

class OGDFNodeImpl {
private:
    ogdf::node node_;
    ogdf::node optPairNode = nullptr;

public:
    OGDFNodeImpl() = default;
    OGDFNodeImpl(ogdf::node n)
        : node_ { n }
    {
    }

    int getId() const { return node_->index(); }
    ogdf::node getNode() const { return node_; }
    ogdf::node getPairNode() const { return optPairNode; }
    void setPairNode(ogdf::node pair) { optPairNode = pair; }
};

class OGDFEdgeImpl {
private:
    ogdf::edge edge_;
    bool virtualEdge;

public:
    OGDFEdgeImpl(ogdf::edge edge, bool isVirtualEdge = false)
        : edge_ { edge }
        , virtualEdge { isVirtualEdge }
    {
    }

    int getId() const { return edge_->index(); }

    ogdf::edge getEdge() const { return edge_; }

    bool isVirtual() const { return virtualEdge; }
};

template <typename NodeStoredObject, typename EdgeStoredObject>
class OGDFGraphImpl {
public:
    using NodeType = NodeInterface<OGDFNodeImpl, NodeStoredObject>;
    using EdgeType = EdgeInterface<OGDFEdgeImpl, EdgeStoredObject>;

    OGDFGraphImpl() = default;
    OGDFGraphImpl(const OGDFGraphImpl&) = delete;
    OGDFGraphImpl& operator=(const OGDFGraphImpl&) = delete;
    OGDFGraphImpl(OGDFGraphImpl&&) noexcept = default;
    OGDFGraphImpl& operator=(OGDFGraphImpl&&) noexcept = default;

private:
    ogdf::Graph graph_;
    std::unordered_map<int, NodeType> nodeMap_;
    std::unordered_map<int, EdgeType> edgeMap_;

    OGDFGraphImpl createMappedSubGraph(
        const std::vector<ogdf::node>& currentlyVisitedNodes,
        const std::vector<ogdf::node>& originalSeparatorNodes,
        bool createVirtual)
    {
        using node = ogdf::node;
        using edge = ogdf::edge;
        getOGDFGraphLogger()->debug(createVirtual);
        ogdf::NodeArray<node> originalToNewNodeMapping(graph_);
        ogdf::EdgeArray<ogdf::edge> originalToNewEdgeMapping(graph_);

        ogdf::List<node> tempList {};
        for (auto n : currentlyVisitedNodes) {
            tempList.pushBack(n);
        }

        OGDFGraphImpl subGraph;

        ogdf::ListIterator<ogdf::node> it { tempList.begin() };
        ogdf::inducedSubGraph(graph_, it, subGraph.graph_,
            originalToNewNodeMapping, originalToNewEdgeMapping);

        for (const auto& n : currentlyVisitedNodes) {
            node inducedNode = originalToNewNodeMapping[n];
            getOGDFGraphLogger()->debug(
                std::format("original node index: {}", n->index()));

            subGraph.nodeMap_.try_emplace(inducedNode->index(),
                NodeType { OGDFNodeImpl { inducedNode },
                    nodeMap_[n->index()].getStoredObj() });
        }

        for (auto& [index, edgeValue] : edgeMap_) {
            auto originalEdge = edgeValue.getImpl().getEdge();
            edge inducedEdge = originalToNewEdgeMapping[originalEdge];

            if (inducedEdge) {
                bool isVirtual = false;

                // TODO currently only separation pairs and cut vertex are
                // supported
                if (originalSeparatorNodes.size() == 2
                    && isEdgeBetweenNodes(originalEdge,
                        originalSeparatorNodes[0], originalSeparatorNodes[1])) {
                    isVirtual = true;
                }

                getOGDFGraphLogger()->debug(std::format(
                    "original edge index: {}", originalEdge->index()));
                if (isVirtual && createVirtual) {
                    getOGDFGraphLogger()->debug("Virtual edge added");
                }

                subGraph.edgeMap_.try_emplace(inducedEdge->index(),
                    EdgeType { OGDFEdgeImpl {
                                   inducedEdge, isVirtual && createVirtual },
                        edgeValue.getStoredObj() });
            }
        }

        return subGraph;
    };

public:
    bool isEdgeBetweenNodes(const ogdf::edge& edgeToCheck,
        const ogdf::node& firstNode, const ogdf::node& secondNode) const
    {
        ogdf::node edgeSource = edgeToCheck->source();
        ogdf::node edgeTarget = edgeToCheck->target();

        return (edgeSource == firstNode && edgeTarget == secondNode)
            || (edgeSource == secondNode && edgeTarget == firstNode);
    }

    NodeType& addNode(std::shared_ptr<NodeStoredObject> obj)
    {
        ogdf::node newNode = graph_.newNode();
        auto newNodeId = newNode->index();

        nodeMap_.emplace(newNodeId, NodeType { OGDFNodeImpl { newNode }, obj });
        return nodeMap_.at(newNodeId);
    }

    EdgeType& addEdge(const NodeType& node1, const NodeType& node2,
        std::shared_ptr<EdgeStoredObject> obj)
    {
        auto it1 = nodeMap_.find(node1.getId());
        auto it2 = nodeMap_.find(node2.getId());

        if (it1 != nodeMap_.end() && it2 != nodeMap_.end()) {
            ogdf::edge newEdge = graph_.newEdge(it1->second.getImpl().getNode(),
                it2->second.getImpl().getNode());
            auto newEdgeId = newEdge->index();

            edgeMap_.emplace(
                newEdgeId, EdgeType { OGDFEdgeImpl { newEdge }, obj });
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

    std::pair<std::optional<NodeType>, std::optional<NodeType>>
    getSeparationPairs() const
    {
        ogdf::node separatorNodeOne;
        ogdf::node separatorNodeTwo;
        bool isTriconnected = false;

        ogdf::Triconnectivity(
            graph_, isTriconnected, separatorNodeOne, separatorNodeTwo);

        if (isTriconnected) {
            getOGDFGraphLogger()->info(
                "Graph is triconnected, no separation pairs exist");
            return { std::nullopt, std::nullopt };
        }

        if (separatorNodeTwo == nullptr) {
            getOGDFGraphLogger()->info("Graph is not biconnected");
            return { std::nullopt, std::nullopt };
        }

        // Debug: Check if keys exist in map
        int index1 = separatorNodeOne->index();
        int index2 = separatorNodeTwo->index();

        auto it1 = nodeMap_.find(index1);
        auto it2 = nodeMap_.find(index2);

        if (it1 == nodeMap_.end()) {
            throw std::logic_error("Node index " + std::to_string(index1)
                + " not found in nodeMap_");
        }
        if (it2 == nodeMap_.end()) {
            throw std::logic_error("Node index " + std::to_string(index2)
                + " not found in nodeMap_");
        }

        return { it1->second, it2->second };
    }

    std::vector<OGDFGraphImpl> separateByVerticesByDuplication(
        const std::vector<NodeType>& separatorNodes)
    {
        using node = ogdf::node;

        std::vector<OGDFGraphImpl> subGraphs {};

        auto numberOfNodes = graph_.nodes.size();
        std::vector<node> ogdfSepNodes {};

        for (auto n : separatorNodes) {
            ogdfSepNodes.push_back(n.getImpl().getNode());
        }

        std::vector<node> absoluteVisitedNodes { ogdfSepNodes };

        while (numberOfNodes != absoluteVisitedNodes.size()) {
            std::vector<node> currentlyVisitedNodes { ogdfSepNodes };
            std::stack<node> nodeStack {};

            // TODO: with large graphs this may be slow af
            auto nextNode = graph_.chooseNode([&absoluteVisitedNodes](node n) {
                return std::ranges::find(absoluteVisitedNodes, n)
                    == absoluteVisitedNodes.end();
            });
            if (nextNode != nullptr) {
                nodeStack.push(nextNode);
            }

            while (!nodeStack.empty()) {
                auto currentNode = nodeStack.top();
                nodeStack.pop();

                bool alreadyInCurrently
                    = std::ranges::find(currentlyVisitedNodes, currentNode)
                    != currentlyVisitedNodes.end();
                bool alreadyInAbsolute
                    = std::ranges::find(absoluteVisitedNodes, currentNode)
                    != absoluteVisitedNodes.end();

                if (alreadyInCurrently || alreadyInAbsolute) {
                    continue;
                }

                currentlyVisitedNodes.push_back(currentNode);
                absoluteVisitedNodes.push_back(currentNode);

                for (auto edge : currentNode->adjEntries) {
                    auto adjNode = edge->twinNode();

                    bool foundInCurrently
                        = std::ranges::find(currentlyVisitedNodes, adjNode)
                        != currentlyVisitedNodes.end();
                    bool foundInAbsolute
                        = std::ranges::find(absoluteVisitedNodes, adjNode)
                        != absoluteVisitedNodes.end();

                    if (!foundInCurrently && !foundInAbsolute) {
                        nodeStack.push(adjNode);
                    }
                }
            }

            // TODO think through virtual edge creation, it may be faulty
            subGraphs.push_back(createMappedSubGraph(
                currentlyVisitedNodes, ogdfSepNodes, subGraphs.size() > 0));
            getOGDFGraphLogger()->debug("subgraph created after separation");
        }

        return subGraphs;
    }

    std::size_t getNodeCount() const { return graph_.numberOfNodes(); }

    std::size_t getEdgeCount() const { return graph_.numberOfEdges(); }
};

static_assert(GraphImplRequirements<OGDFGraphImpl<int, int>, OGDFNodeImpl, int,
    OGDFEdgeImpl, int>);

template <typename NodeStoredObj, typename EdgeStoredObj>
using Graph = GraphInterface<OGDFGraphImpl<NodeStoredObj, EdgeStoredObj>,
    OGDFNodeImpl, NodeStoredObj, OGDFEdgeImpl, EdgeStoredObj>;
} // namespace MathUtils

#endif // OGDF_GRAPH_WRAPPER_HPP
