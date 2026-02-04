#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <concepts>
#include <functional>
#include <optional>
#include <ranges>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace MathUtils {

// ============================================================
// Helper concepts
// ============================================================

// TODO: factor out to a general utility header
template <typename T, std::size_t N>
concept BindableTo = requires {
    typename std::tuple_size<T>::type;
    requires std::tuple_size_v<T> == N;
};

// ============================================================
// Identity concepts
// ============================================================

template <typename N>
concept NodeIdentity = std::regular<N> && requires(N n) {
    { std::hash<N> {}(n) } -> std::convertible_to<std::size_t>;
};

template <typename E>
concept EdgeIdentity = std::regular<E> && requires(E e) {
    { std::hash<E> {}(e) } -> std::convertible_to<std::size_t>;
};

// ============================================================
// Graph concepts
// ============================================================

template <typename G>
concept GraphBase = NodeIdentity<typename G::NodeIdType>
    && EdgeIdentity<typename G::EdgeIdType>
    && requires(G g, typename G::NodeIdType n, typename G::EdgeIdType e) {
           requires(
               !std::same_as<typename G::NodeIdType, typename G::EdgeIdType>);

           { g.getNodes() } -> std::ranges::range;
           { g.getEdges() } -> std::ranges::range;
           { g.getEdges(n) } -> std::ranges::range;
           { g.getNeighbors(n) } -> std::ranges::range;
           { g.getEndpoints(e) } -> BindableTo<2>;
       };

template <typename G>
concept GraphTopology = GraphBase<G>
    && requires(G g, typename G::NodeIdType n, typename G::EdgeIdType e) {
           { g.addNode() } -> std::same_as<typename G::NodeIdType>;
           { g.addEdge(n, n) } -> std::same_as<typename G::EdgeIdType>;
           { g.removeNode(n) };
           { g.removeEdge(e) };
       };

// ============================================================
// Proxy templates
// ============================================================

template <GraphBase G>
class NodeProxy {
public:
    using NodeId = typename G::NodeIdType;
    using EdgeId = typename G::EdgeIdType;

    NodeProxy() = default;

    NodeProxy(NodeId id, const G& graph)
        : m_id(id)
        , m_graph(&graph)
    {
    }

    auto getNeighbors() const { return m_graph->getNeighbors(m_id); }

    auto getEdges() const { return m_graph->getEdges(m_id); }

    NodeId id() const { return m_id; }
    operator NodeId() const { return m_id; }

    bool operator==(const NodeProxy& other) const { return m_id == other.m_id; }

private:
    NodeId m_id {};
    const G* m_graph { nullptr };
};

template <GraphBase G>
class EdgeProxy {
public:
    using NodeId = typename G::NodeIdType;
    using EdgeId = typename G::EdgeIdType;

    EdgeProxy() = default;

    EdgeProxy(EdgeId id, const G& graph)
        : m_id(id)
        , m_graph(&graph)
    {
    }

    auto getEndpoints() const { return m_graph->getEndpoints(m_id); }

    EdgeId id() const { return m_id; }
    operator EdgeId() const { return m_id; }

    bool operator==(const EdgeProxy& other) const { return m_id == other.m_id; }

private:
    EdgeId m_id {};
    const G* m_graph { nullptr };
};

template <GraphBase G>
class GraphView {
public:
    using NodeIdType = typename G::NodeIdType;
    using EdgeIdType = typename G::EdgeIdType;

    GraphView(const G& graph, std::unordered_set<NodeIdType> subset)
        : m_parentGraph(&graph)
        , m_nodeSubset(std::move(subset))
    {
    }

    const std::unordered_set<NodeIdType>& getNodes() const
    {
        return m_nodeSubset;
    }

    auto getEdges() const
    {
        return m_parentGraph->getEdges()
            | std::views::filter([this](const EdgeIdType& e) {
                  auto [s, t] = m_parentGraph->getEndpoints(e);
                  return m_nodeSubset.contains(s) && m_nodeSubset.contains(t);
              });
    }

    auto getEdges(NodeIdType n) const
    {
        return m_parentGraph->getEdges(n)
            | std::views::filter([this](const EdgeIdType& e) {
                  auto [s, t] = m_parentGraph->getEndpoints(e);
                  return m_nodeSubset.contains(s) && m_nodeSubset.contains(t);
              });
    }

    auto getNeighbors(NodeIdType n) const
    {
        return m_parentGraph->getNeighbors(n)
            | std::views::filter([this](const NodeIdType& neighbor) {
                  return m_nodeSubset.contains(neighbor);
              });
    }

    auto getEndpoints(EdgeIdType e) const
    {
        return m_parentGraph->getEndpoints(e);
    }

private:
    const G* m_parentGraph;
    std::unordered_set<NodeIdType> m_nodeSubset;
};

// ============================================================
// GraphOverlay
// ============================================================

template <GraphTopology G>
class GraphOverlay {
public:
    using NodeIdType = typename G::NodeIdType;
    using EdgeIdType = typename G::EdgeIdType;

    GraphOverlay(G& graph, std::unordered_set<NodeIdType> subset)
        : m_parentGraph(&graph)
        , m_nodeSubset(std::move(subset))
    {
    }

    const std::unordered_set<NodeIdType>& getNodes() const
    {
        return m_nodeSubset;
    }

    auto getEdges() const
    {
        return m_parentGraph->getEdges()
            | std::views::filter([this](const EdgeIdType& e) {
                  auto [s, t] = m_parentGraph->getEndpoints(e);
                  return m_nodeSubset.contains(s) && m_nodeSubset.contains(t);
              });
    }

    auto getEdges(NodeIdType n) const
    {
        return m_parentGraph->getEdges(n)
            | std::views::filter([this](const EdgeIdType& e) {
                  auto [s, t] = m_parentGraph->getEndpoints(e);
                  return m_nodeSubset.contains(s) && m_nodeSubset.contains(t);
              });
    }

    auto getNeighbors(NodeIdType n) const
    {
        return m_parentGraph->getNeighbors(n)
            | std::views::filter([this](const NodeIdType& neighbor) {
                  return m_nodeSubset.contains(neighbor);
              });
    }

    auto getEndpoints(EdgeIdType e) const
    {
        return m_parentGraph->getEndpoints(e);
    }

    NodeIdType addNode()
    {
        auto n = m_parentGraph->addNode();
        m_nodeSubset.insert(n);
        return n;
    }

    EdgeIdType addEdge(NodeIdType s, NodeIdType t)
    {
        return m_parentGraph->addEdge(s, t);
    }

    void removeNode(NodeIdType n)
    {
        m_parentGraph->removeNode(n);
        m_nodeSubset.erase(n);
    }

    void removeEdge(EdgeIdType e) { m_parentGraph->removeEdge(e); }

private:
    G* m_parentGraph;
    std::unordered_set<NodeIdType> m_nodeSubset;
};

// ============================================================
// SubGraph
// ============================================================

template <GraphTopology G>
class SubGraph {
public:
    using NodeIdType = typename G::NodeIdType;
    using EdgeIdType = typename G::EdgeIdType;

    static SubGraph extract(
        const G& parent, const std::unordered_set<NodeIdType>& subset)
    {
        SubGraph sub;

        for (const auto& origNode : subset) {
            auto localNode = sub.m_graph.addNode();
            sub.m_originalToLocalNode[origNode] = localNode;
            sub.m_localToOriginalNode[localNode] = origNode;
        }

        for (const auto& origEdge : parent.getEdges()) {
            auto [s, t] = parent.getEndpoints(origEdge);
            if (subset.contains(s) && subset.contains(t)) {
                auto localEdge
                    = sub.m_graph.addEdge(sub.m_originalToLocalNode.at(s),
                        sub.m_originalToLocalNode.at(t));
                sub.m_originalToLocalEdge[origEdge] = localEdge;
                sub.m_localToOriginalEdge[localEdge] = origEdge;
            }
        }

        return sub;
    }

    auto getNodes() const { return m_graph.getNodes(); }
    auto getEdges() const { return m_graph.getEdges(); }

    auto getEdges(NodeIdType n) const { return m_graph.getEdges(n); }

    auto getNeighbors(NodeIdType n) const { return m_graph.getNeighbors(n); }

    auto getEndpoints(EdgeIdType e) const { return m_graph.getEndpoints(e); }

    NodeIdType addNode() { return m_graph.addNode(); }

    EdgeIdType addEdge(NodeIdType s, NodeIdType t)
    {
        return m_graph.addEdge(s, t);
    }

    void removeNode(NodeIdType n)
    {
        if (auto it = m_localToOriginalNode.find(n);
            it != m_localToOriginalNode.end()) {
            m_originalToLocalNode.erase(it->second);
            m_localToOriginalNode.erase(it);
        }
        m_graph.removeNode(n);
    }

    void removeEdge(EdgeIdType e)
    {
        if (auto it = m_localToOriginalEdge.find(e);
            it != m_localToOriginalEdge.end()) {
            m_originalToLocalEdge.erase(it->second);
            m_localToOriginalEdge.erase(it);
        }
        m_graph.removeEdge(e);
    }

    // TODO: Note that these could use the new C++23 API, but we would require
    // valid error codes for that
    std::optional<NodeIdType> originalNodeId(NodeIdType local) const
    {
        auto it = m_localToOriginalNode.find(local);
        if (it != m_localToOriginalNode.end())
            return it->second;
        return std::nullopt;
    }

    std::optional<NodeIdType> localNodeId(NodeIdType original) const
    {
        auto it = m_originalToLocalNode.find(original);
        if (it != m_originalToLocalNode.end())
            return it->second;
        return std::nullopt;
    }

    std::optional<EdgeIdType> originalEdgeId(EdgeIdType local) const
    {
        auto it = m_localToOriginalEdge.find(local);
        if (it != m_localToOriginalEdge.end())
            return it->second;
        return std::nullopt;
    }

    std::optional<EdgeIdType> localEdgeId(EdgeIdType original) const
    {
        auto it = m_originalToLocalEdge.find(original);
        if (it != m_originalToLocalEdge.end())
            return it->second;
        return std::nullopt;
    }

private:
    SubGraph() = default;

    G m_graph;

    std::unordered_map<NodeIdType, NodeIdType> m_originalToLocalNode;
    std::unordered_map<NodeIdType, NodeIdType> m_localToOriginalNode;
    std::unordered_map<EdgeIdType, EdgeIdType> m_originalToLocalEdge;
    std::unordered_map<EdgeIdType, EdgeIdType> m_localToOriginalEdge;
};

} // namespace MathUtils

// ============================================================
// std::hash specializations for proxy types
// ============================================================

template <MathUtils::GraphBase G>
struct std::hash<MathUtils::NodeProxy<G>> {
    std::size_t operator()(const MathUtils::NodeProxy<G>& proxy) const noexcept
    {
        return std::hash<typename G::NodeIdType> {}(proxy.id());
    }
};

template <MathUtils::GraphBase G>
struct std::hash<MathUtils::EdgeProxy<G>> {
    std::size_t operator()(const MathUtils::EdgeProxy<G>& proxy) const noexcept
    {
        return std::hash<typename G::EdgeIdType> {}(proxy.id());
    }
};

#endif
