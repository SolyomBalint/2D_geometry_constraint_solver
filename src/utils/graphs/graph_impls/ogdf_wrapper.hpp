/**
 * WARNING: This file is AI generated and is not tested. This wrapper is only
 * used for rapid prototyping
 */

#ifndef OGDF_GRAPH_WRAPPER_HPP
#define OGDF_GRAPH_WRAPPER_HPP

#include "../graph_interface.hpp"
#include <algorithm>
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
#include <unordered_set>

namespace MathUtils {

inline std::shared_ptr<spdlog::logger> getOGDFGraphLogger()
{
    static auto logger = spdlog::stdout_color_mt("OGDF_GRAPH");
    return logger;
}

class OGDFNodeImpl {
private:
    Common::Uuid id_;

public:
    OGDFNodeImpl()
        : id_(Common::generateUuidMt19937())
    {
    }

    Common::Uuid getId() const { return id_; }

    bool operator==(const OGDFNodeImpl& other) const
    {
        return id_ == other.id_;
    }
};

class OGDFEdgeImpl {
private:
    Common::Uuid id_;
    bool virtualEdge_;

public:
    OGDFEdgeImpl(bool isVirtual = false)
        : id_(Common::generateUuidMt19937())
        , virtualEdge_(isVirtual)
    {
    }

    Common::Uuid getId() const { return id_; }

    bool isVirtual() const { return virtualEdge_; }

    bool operator==(const OGDFEdgeImpl& other) const
    {
        return id_ == other.id_;
    }
};

template <typename NodeStoredObject, typename EdgeStoredObject>
class OGDFGraphImpl {
public:
    using NodeType = NodeInterface<OGDFNodeImpl, NodeStoredObject>;
    using EdgeType = EdgeInterface<OGDFEdgeImpl, EdgeStoredObject>;

    OGDFGraphImpl() = default;

    OGDFGraphImpl(const OGDFGraphImpl& other) = default;
    OGDFGraphImpl& operator=(const OGDFGraphImpl& other) = default;
    OGDFGraphImpl(OGDFGraphImpl&&) noexcept = default;
    OGDFGraphImpl& operator=(OGDFGraphImpl&&) noexcept = default;

private:
    // Custom graph storage - no persistent OGDF state
    std::unordered_map<Common::Uuid, NodeType> nodes_;
    std::unordered_map<Common::Uuid, EdgeType> edges_;

    // Edge connectivity: edge UUID -> {source UUID, target UUID}
    struct EdgeConnection {
        Common::Uuid source;
        Common::Uuid target;
    };
    std::unordered_map<Common::Uuid, EdgeConnection> edgeConnections_;

    // Adjacency list: node UUID -> set of edge UUIDs
    std::unordered_map<Common::Uuid, std::unordered_set<Common::Uuid>>
        adjacency_;

    // Temporary OGDF conversion structures
    struct OGDFConversion {
        ogdf::Graph graph;
        std::unordered_map<Common::Uuid, ogdf::node> uuidToNode;
        std::unordered_map<ogdf::node, Common::Uuid> nodeToUuid;
        std::unordered_map<Common::Uuid, ogdf::edge> uuidToEdge;
        std::unordered_map<ogdf::edge, Common::Uuid> edgeToUuid;
    };

    // Convert our graph to OGDF format for algorithm execution
    OGDFConversion toOGDFGraph() const
    {
        OGDFConversion conv;

        // Create OGDF nodes
        for (const auto& [uuid, node] : nodes_) {
            ogdf::node ogdfNode = conv.graph.newNode();
            conv.uuidToNode[uuid] = ogdfNode;
            conv.nodeToUuid[ogdfNode] = uuid;
        }

        // Create OGDF edges
        for (const auto& [uuid, edge] : edges_) {
            const auto& conn = edgeConnections_.at(uuid);
            ogdf::node ogdfSource = conv.uuidToNode.at(conn.source);
            ogdf::node ogdfTarget = conv.uuidToNode.at(conn.target);
            ogdf::edge ogdfEdge = conv.graph.newEdge(ogdfSource, ogdfTarget);
            conv.uuidToEdge[uuid] = ogdfEdge;
            conv.edgeToUuid[ogdfEdge] = uuid;
        }

        return conv;
    }

    // Create subgraph from a set of node UUIDs
    OGDFGraphImpl createSubgraph(
        const std::unordered_set<Common::Uuid>& nodeUuids,
        const std::vector<Common::Uuid>& separatorUuids,
        std::shared_ptr<EdgeStoredObject> virtualEdgeObj) const
    {
        OGDFGraphImpl subgraph;

        // Copy nodes
        for (const auto& uuid : nodeUuids) {
            auto it = nodes_.find(uuid);
            if (it != nodes_.end()) {
                subgraph.nodes_[uuid] = it->second;
            }
        }

        // Copy edges that have both endpoints in the subgraph
        for (const auto& [edgeUuid, edge] : edges_) {
            const auto& conn = edgeConnections_.at(edgeUuid);

            if (nodeUuids.contains(conn.source)
                && nodeUuids.contains(conn.target)) {
                // Copy edge as-is (preserve original virtual flag)
                subgraph.edges_[edgeUuid] = edge;
                subgraph.edgeConnections_[edgeUuid] = conn;

                // Update adjacency
                subgraph.adjacency_[conn.source].insert(edgeUuid);
                subgraph.adjacency_[conn.target].insert(edgeUuid);
            }
        }

        // Add virtual edge between separators if none exists and virtual edge
        // object provided
        if (separatorUuids.size() == 2 && virtualEdgeObj != nullptr) {
            Common::Uuid sep1 = separatorUuids[0];
            Common::Uuid sep2 = separatorUuids[1];

            // Check if there's already an edge between the separators
            bool edgeExists = false;
            auto adjIt = subgraph.adjacency_.find(sep1);
            if (adjIt != subgraph.adjacency_.end()) {
                for (const auto& edgeUuid : adjIt->second) {
                    const auto& conn = subgraph.edgeConnections_.at(edgeUuid);
                    if ((conn.source == sep1 && conn.target == sep2)
                        || (conn.source == sep2 && conn.target == sep1)) {
                        edgeExists = true;
                        break;
                    }
                }
            }

            // Create virtual edge if no edge exists TODO: this is not the full
            // laman check
            bool isWellconstrained
                = subgraph.getEdgeCount() == 2 * subgraph.getNodeCount() - 3;
            bool isBiconnected = subgraph.getCutVertices().empty();
            if (!isBiconnected) {
                EdgeType virtualEdge(OGDFEdgeImpl(true), virtualEdgeObj);
                Common::Uuid edgeId = virtualEdge.getId();

                subgraph.edges_[edgeId] = virtualEdge;
                subgraph.edgeConnections_[edgeId] = { sep1, sep2 };

                // Update adjacency lists
                subgraph.adjacency_[sep1].insert(edgeId);
                subgraph.adjacency_[sep2].insert(edgeId);
            }
        }

        return subgraph;
    }

public:
    NodeType& addNode(std::shared_ptr<NodeStoredObject> obj)
    {
        NodeType newNode(OGDFNodeImpl(), obj);
        Common::Uuid nodeId = newNode.getId();
        nodes_[nodeId] = newNode;
        adjacency_[nodeId] = {}; // Initialize empty adjacency set
        return nodes_.at(nodeId);
    }

    EdgeType& addEdge(const NodeType& node1, const NodeType& node2,
        std::shared_ptr<EdgeStoredObject> obj)
    {
        auto it1 = nodes_.find(node1.getId());
        auto it2 = nodes_.find(node2.getId());

        if (it1 == nodes_.end() || it2 == nodes_.end()) {
            throw std::runtime_error("Invalid nodes for edge creation");
        }

        EdgeType newEdge(OGDFEdgeImpl(false), obj);
        Common::Uuid edgeId = newEdge.getId();

        edges_[edgeId] = newEdge;
        edgeConnections_[edgeId] = { node1.getId(), node2.getId() };

        // Update adjacency lists
        adjacency_[node1.getId()].insert(edgeId);
        adjacency_[node2.getId()].insert(edgeId);

        return edges_.at(edgeId);
    }

    std::vector<NodeType> getCutVertices() const
    {
        // Convert to OGDF for algorithm execution
        auto conv = toOGDFGraph();

        // Find cut vertices using OGDF
        ogdf::ArrayBuffer<ogdf::node> cutVertices;
        ogdf::findCutVertices(conv.graph, cutVertices);

        // Convert results back to our node types
        std::vector<NodeType> outNodes;
        for (auto& ogdfNode : cutVertices) {
            Common::Uuid nodeUuid = conv.nodeToUuid.at(ogdfNode);
            outNodes.push_back(nodes_.at(nodeUuid));
        }

        return outNodes;
    }

    std::pair<std::optional<NodeType>, std::optional<NodeType>>
    getSeparationPairs() const
    {
        // Convert to OGDF for algorithm execution
        auto conv = toOGDFGraph();

        ogdf::node separatorNodeOne;
        ogdf::node separatorNodeTwo;
        bool isTriconnected = false;

        ogdf::Triconnectivity(
            conv.graph, isTriconnected, separatorNodeOne, separatorNodeTwo);

        if (isTriconnected) {
            getOGDFGraphLogger()->info(
                "Graph is triconnected, no separation pairs exist");
            return { std::nullopt, std::nullopt };
        }

        if (separatorNodeTwo == nullptr) {
            getOGDFGraphLogger()->info("Graph is not biconnected");
            return { std::nullopt, std::nullopt };
        }

        // Convert OGDF nodes back to our node types
        Common::Uuid uuid1 = conv.nodeToUuid.at(separatorNodeOne);
        Common::Uuid uuid2 = conv.nodeToUuid.at(separatorNodeTwo);

        return { nodes_.at(uuid1), nodes_.at(uuid2) };
    }

    std::vector<OGDFGraphImpl> separateByVerticesByDuplication(
        const std::vector<NodeType>& separatorNodes,
        std::shared_ptr<EdgeStoredObject> virtualEdgeObj = nullptr)
    {
        std::vector<OGDFGraphImpl> subGraphs;
        std::vector<std::unordered_set<Common::Uuid>> componentNodeSets;

        // Convert separator nodes to UUIDs
        std::vector<Common::Uuid> separatorUuids;
        std::unordered_set<Common::Uuid> separatorSet;
        for (const auto& n : separatorNodes) {
            separatorUuids.push_back(n.getId());
            separatorSet.insert(n.getId());
        }

        // Track all visited nodes across all components
        std::unordered_set<Common::Uuid> absoluteVisited(separatorSet);

        size_t totalNodes = nodes_.size();

        // Find connected components (excluding separators from traversal
        // boundaries)
        while (absoluteVisited.size() < totalNodes) {
            // Find an unvisited node to start DFS
            Common::Uuid startNode;
            bool found = false;
            for (const auto& [uuid, node] : nodes_) {
                if (!absoluteVisited.contains(uuid)) {
                    startNode = uuid;
                    found = true;
                    break;
                }
            }

            if (!found) {
                break;
            }

            // DFS to find connected component
            std::unordered_set<Common::Uuid> componentNodes(separatorSet);
            std::stack<Common::Uuid> nodeStack;
            nodeStack.push(startNode);

            while (!nodeStack.empty()) {
                Common::Uuid currentUuid = nodeStack.top();
                nodeStack.pop();

                if (componentNodes.contains(currentUuid)
                    || absoluteVisited.contains(currentUuid)) {
                    continue;
                }

                componentNodes.insert(currentUuid);
                absoluteVisited.insert(currentUuid);

                // Explore neighbors through edges
                auto adjIt = adjacency_.find(currentUuid);
                if (adjIt != adjacency_.end()) {
                    for (const auto& edgeUuid : adjIt->second) {
                        const auto& conn = edgeConnections_.at(edgeUuid);

                        // Find the neighbor node
                        Common::Uuid neighborUuid = (conn.source == currentUuid)
                            ? conn.target
                            : conn.source;

                        // Add to stack if not visited and not a separator
                        if (!absoluteVisited.contains(neighborUuid)
                            && !componentNodes.contains(neighborUuid)) {
                            nodeStack.push(neighborUuid);
                        }
                    }
                }
            }

            componentNodeSets.push_back(componentNodes);
        }

        for (const auto& componentNodes : componentNodeSets) {
            subGraphs.push_back(
                createSubgraph(componentNodes, separatorUuids, virtualEdgeObj));
        }

        return subGraphs;
    }

    std::size_t getNodeCount() const { return nodes_.size(); }

    std::size_t getEdgeCount() const { return edges_.size(); }

    std::vector<NodeType> getNodes() const
    {
        std::vector<NodeType> nodes;
        nodes.reserve(nodes_.size());

        for (const auto& [uuid, node] : nodes_) {
            nodes.push_back(node);
        }
        return nodes;
    }

    std::vector<EdgeType> getEdges() const
    {
        std::vector<EdgeType> edges;
        edges.reserve(edges_.size());

        for (const auto& [uuid, edge] : edges_) {
            edges.push_back(edge);
        }
        return edges;
    }

    std::optional<EdgeType> getEdgeBetweenNodes(
        const NodeType& node1, const NodeType& node2) const
    {
        Common::Uuid uuid1 = node1.getId();
        Common::Uuid uuid2 = node2.getId();

        // Check if both nodes exist
        if (!nodes_.contains(uuid1) || !nodes_.contains(uuid2)) {
            return std::nullopt;
        }

        // Look through edges adjacent to node1
        auto adjIt = adjacency_.find(uuid1);
        if (adjIt == adjacency_.end()) {
            return std::nullopt;
        }

        // Check each edge to see if it connects to node2
        for (const auto& edgeUuid : adjIt->second) {
            const auto& conn = edgeConnections_.at(edgeUuid);

            if ((conn.source == uuid1 && conn.target == uuid2)
                || (conn.source == uuid2 && conn.target == uuid1)) {
                return edges_.at(edgeUuid);
            }
        }

        return std::nullopt;
    }

    bool hasVirtualEdge() const
    {
        return std::ranges::any_of(
            edges_, [](const auto& pair) { return pair.second.isVirtual(); });
    }
};

static_assert(GraphImplRequirements<OGDFGraphImpl<int, int>, OGDFNodeImpl, int,
    OGDFEdgeImpl, int>);

template <typename NodeStoredObj, typename EdgeStoredObj>
using Graph = GraphInterface<OGDFGraphImpl<NodeStoredObj, EdgeStoredObj>,
    OGDFNodeImpl, NodeStoredObj, OGDFEdgeImpl, EdgeStoredObj>;
} // namespace MathUtils

#endif // OGDF_GRAPH_WRAPPER_HPP
