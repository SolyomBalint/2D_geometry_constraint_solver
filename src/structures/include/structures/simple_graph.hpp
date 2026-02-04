#ifndef SIMPLE_GRAPH_HPP
#define SIMPLE_GRAPH_HPP

// Custom headers
#include <structures/graph.hpp>
#include <structures/graph_errors.hpp>

// General STD/STL headers
#include <algorithm>
#include <cassert>
#include <compare>
#include <cstddef>
#include <expected>
#include <functional>
#include <ranges>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace MathUtils {

struct NodeId {
    int value {};

    bool operator==(const NodeId&) const = default;
    auto operator<=>(const NodeId&) const = default;
};

struct EdgeId {
    int value {};

    bool operator==(const EdgeId&) const = default;
    auto operator<=>(const EdgeId&) const = default;
};

} // namespace MathUtils

template <>
struct std::hash<MathUtils::NodeId> {
    std::size_t operator()(const MathUtils::NodeId& id) const noexcept
    {
        return std::hash<int> {}(id.value);
    }
};

template <>
struct std::hash<MathUtils::EdgeId> {
    std::size_t operator()(const MathUtils::EdgeId& id) const noexcept
    {
        return std::hash<int> {}(id.value);
    }
};

namespace MathUtils {

// ============================================================
// SimpleGraph
// ============================================================
// Storage:
//   m_adjacency     : NodeId -> set of incident EdgeIds
//   m_edgeEndpoints : EdgeId -> pair<NodeId, NodeId>
//

class SimpleGraph {
public:
    using NodeIdType = NodeId;
    using EdgeIdType = EdgeId;

    SimpleGraph() = default;

    auto getNodes() const { return std::views::keys(m_adjacency); }

    auto getEdges() const { return std::views::keys(m_edgeEndpoints); }

    /// @pre @p n must be a valid node in this graph.
    const std::unordered_set<EdgeId>& getEdges(NodeId n) const
    {
        auto it = m_adjacency.find(n);
        assert(it != m_adjacency.end() && "getEdges: invalid NodeId");
        return it->second;
    }

    auto getNeighbors(NodeId n) const
    {
        return getEdges(n) | std::views::transform([this, n](const EdgeId& e) {
            auto [s, t] = getEndpoints(e);
            return (s == n) ? t : s;
        });
    }

    /// @pre @p e must be a valid edge in this graph.
    std::pair<NodeId, NodeId> getEndpoints(EdgeId e) const
    {
        auto it = m_edgeEndpoints.find(e);
        assert(it != m_edgeEndpoints.end() && "getEndpoints: invalid EdgeId");
        return it->second;
    }

    NodeId addNode()
    {
        NodeId id { m_nextNodeId++ };
        m_adjacency[id];
        return id;
    }

    std::expected<EdgeId, GraphError> addEdge(NodeId s, NodeId t)
    {
        if (!hasNode(s) || !hasNode(t))
            return std::unexpected(GraphError::NodeNotFound);

        EdgeId id { m_nextEdgeId++ };
        m_edgeEndpoints[id] = { s, t };
        m_adjacency[s].insert(id);
        m_adjacency[t].insert(id);
        return id;
    }

    std::expected<void, GraphError> removeNode(NodeId n)
    {
        auto it = m_adjacency.find(n);
        if (it == m_adjacency.end())
            return std::unexpected(GraphError::NodeNotFound);

        auto incidentEdges
            = std::vector<EdgeId>(it->second.begin(), it->second.end());

        for (const auto& e : incidentEdges)
            removeEdge(e);

        m_adjacency.erase(n);
        return {};
    }

    std::expected<void, GraphError> removeEdge(EdgeId e)
    {
        auto it = m_edgeEndpoints.find(e);
        if (it == m_edgeEndpoints.end())
            return std::unexpected(GraphError::EdgeNotFound);

        auto [s, t] = it->second;
        m_adjacency[s].erase(e);
        m_adjacency[t].erase(e);
        m_edgeEndpoints.erase(it);
        return {};
    }

    std::size_t nodeCount() const { return m_adjacency.size(); }
    std::size_t edgeCount() const { return m_edgeEndpoints.size(); }

    bool hasNode(NodeId n) const { return m_adjacency.contains(n); }
    bool hasEdge(EdgeId e) const { return m_edgeEndpoints.contains(e); }

    bool hasEdgeBetween(NodeId s, NodeId t) const
    {
        auto itS = m_adjacency.find(s);
        if (itS == m_adjacency.end() || !m_adjacency.contains(t))
            return false;

        for (const auto& e : itS->second) {
            auto [a, b] = m_edgeEndpoints.at(e);
            if ((a == s && b == t) || (a == t && b == s))
                return true;
        }
        return false;
    }

    std::expected<EdgeId, GraphError> getEdgeBetween(NodeId s, NodeId t) const
    {
        auto itS = m_adjacency.find(s);
        if (itS == m_adjacency.end() || !m_adjacency.contains(t))
            return std::unexpected(GraphError::EdgeNotFound);

        for (const auto& e : itS->second) {
            auto [a, b] = m_edgeEndpoints.at(e);
            if ((a == s && b == t) || (a == t && b == s))
                return e;
        }
        return std::unexpected(GraphError::EdgeNotFound);
    }

private:
    int m_nextNodeId { 0 };
    int m_nextEdgeId { 0 };

    std::unordered_map<NodeId, std::unordered_set<EdgeId>> m_adjacency;
    std::unordered_map<EdgeId, std::pair<NodeId, NodeId>> m_edgeEndpoints;
};

static_assert(GraphBase<SimpleGraph>, "SimpleGraph must satisfy GraphBase");
static_assert(
    GraphTopology<SimpleGraph>, "SimpleGraph must satisfy GraphTopology");

} // namespace MathUtils

#endif // SIMPLE_GRAPH_HPP
