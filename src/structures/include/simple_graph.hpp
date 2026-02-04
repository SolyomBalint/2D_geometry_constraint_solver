#ifndef SIMPLE_GRAPH_HPP
#define SIMPLE_GRAPH_HPP

#include "graph.hpp"

#include <algorithm>
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
};

struct EdgeId {
    int value {};

    bool operator==(const EdgeId&) const = default;
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

    const std::unordered_set<EdgeId>& getEdges(NodeId n) const
    {
        static const std::unordered_set<EdgeId> empty {};
        auto it = m_adjacency.find(n);
        if (it != m_adjacency.end())
            return it->second;
        return empty;
    }

    auto getNeighbors(NodeId n) const
    {
        return getEdges(n) | std::views::transform([this, n](const EdgeId& e) {
            auto [s, t] = getEndpoints(e);
            return (s == n) ? t : s;
        });
    }

    std::pair<NodeId, NodeId> getEndpoints(EdgeId e) const
    {
        return m_edgeEndpoints.at(e);
    }

    NodeId addNode()
    {
        NodeId id { m_nextNodeId++ };
        m_adjacency[id];
        return id;
    }

    EdgeId addEdge(NodeId s, NodeId t)
    {
        EdgeId id { m_nextEdgeId++ };
        m_edgeEndpoints[id] = { s, t };
        m_adjacency[s].insert(id);
        m_adjacency[t].insert(id);
        return id;
    }

    void removeNode(NodeId n)
    {
        auto it = m_adjacency.find(n);
        if (it == m_adjacency.end())
            return;

        auto incidentEdges
            = std::vector<EdgeId>(it->second.begin(), it->second.end());

        for (const auto& e : incidentEdges)
            removeEdge(e);

        m_adjacency.erase(n);
    }

    void removeEdge(EdgeId e)
    {
        auto it = m_edgeEndpoints.find(e);
        if (it == m_edgeEndpoints.end())
            return;

        auto [s, t] = it->second;
        m_adjacency[s].erase(e);
        m_adjacency[t].erase(e);
        m_edgeEndpoints.erase(it);
    }

    std::size_t nodeCount() const { return m_adjacency.size(); }
    std::size_t edgeCount() const { return m_edgeEndpoints.size(); }

    bool hasNode(NodeId n) const { return m_adjacency.contains(n); }
    bool hasEdge(EdgeId e) const { return m_edgeEndpoints.contains(e); }

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
