#ifndef DEFAULT_HPP
#define DEFAULT_HPP

#include "../graph_interface.hpp"
#include <common/common_uuid.hpp>
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <unordered_set>
#include <utility>
#include <unordered_map>
#include <vector>
#include <stdexcept>

namespace MathUtils {

const auto GRAPH_LOGGER = spdlog::stdout_color_mt("GRAPH");

class DefaultNode final {
private:
    Common::Uuid id;

public:
    using NodeId = Common::Uuid;

    DefaultNode() = delete;
    DefaultNode(NodeId id)
        : id { id }
    {
    }

    DefaultNode(const DefaultNode&) = default;
    DefaultNode& operator=(const DefaultNode&) = default;
    DefaultNode(DefaultNode&&) = default;
    DefaultNode& operator=(DefaultNode&&) = default;

    const Common::Uuid& getId() const { return id; }

    bool operator==(const DefaultNode& other) const { return id == other.id; }
};

class DefaultEdge final {
private:
    Common::Uuid id;

public:
    using EdgeId = Common::Uuid;

    DefaultEdge() = delete;
    DefaultEdge(EdgeId id)
        : id { id }
    {
    }

    DefaultEdge(const DefaultEdge&) = default;
    DefaultEdge& operator=(const DefaultEdge&) = default;
    DefaultEdge(DefaultEdge&&) = default;
    DefaultEdge& operator=(DefaultEdge&&) = default;

    const Common::Uuid& getId() const { return id; }

    bool operator==(const DefaultEdge& other) const { return id == other.id; }
};

template <typename NodeStoredObject, typename EdgeStoredObject> class DefaultUndirectedGraph {
public:
    using NodeType = NodeInterface<DefaultNode, NodeStoredObject>;
    using EdgeType = EdgeInterface<DefaultEdge, EdgeStoredObject>;
    DefaultUndirectedGraph() = default;

    DefaultUndirectedGraph(const DefaultUndirectedGraph&) = delete;
    DefaultUndirectedGraph& operator=(const DefaultUndirectedGraph&) = delete;
    DefaultUndirectedGraph(DefaultUndirectedGraph&&) = delete;
    DefaultUndirectedGraph& operator=(DefaultUndirectedGraph&&) = delete;

    NodeType& addNode(std::shared_ptr<NodeStoredObject> obj);
    EdgeType& addEdge(const NodeType& node1, const NodeType& node2, std::shared_ptr<EdgeStoredObject> edgeObj);

    std::vector<NodeType> getCutVertices() const;

    std::pair<NodeType&, NodeType&> getSeparationPairs() const
    {
        // Skeleton implementation
        throw std::logic_error("Method not implemented");
    }

private:
    // Adjacency list with explicit access
    std::unordered_map<DefaultNode::NodeId, std::unordered_set<DefaultEdge::EdgeId>> adjacencyLists;
    std::unordered_map<DefaultEdge::EdgeId, std::pair<DefaultNode::NodeId, DefaultNode::NodeId>> neighbours;

    std::unordered_map<DefaultNode::NodeId, NodeType> nodes;
    std::unordered_map<DefaultEdge::EdgeId, EdgeType> edges;
};

static_assert(GraphImplRequirements<DefaultUndirectedGraph<int, int>, DefaultNode, int, DefaultEdge, int>);

} // namespace MathUtils

#include "default.tpp"

#endif // DEFAULT_HPP
