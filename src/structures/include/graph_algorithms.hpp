#ifndef GRAPH_ALGORITHMS_HPP
#define GRAPH_ALGORITHMS_HPP

#include "graph.hpp"

#include <algorithm>
#include <cstddef>
#include <ranges>
#include <unordered_map>
#include <vector>

namespace MathUtils {

/**
 * @brief Check whether a graph is biconnected.
 *
 * A graph is biconnected if it is connected, has no articulation points,
 * and contains at least 2 vertices. Uses Tarjan's algorithm with lowpoint
 * tracking.
 *
 * @tparam G A type satisfying the GraphBase concept.
 * @param graph The input graph to test.
 * @return @c true if the graph is biconnected, @c false otherwise.
 *
 * @par Complexity
 * O(V + E)
 */
template <GraphBase G>
bool isBiconnected(const G& graph)
{
    using NodeId = typename G::NodeIdType;

    // Collect nodes into a vector for indexing
    std::vector<NodeId> nodes;
    for (const auto& n : graph.getNodes())
        nodes.push_back(n);

    int n = static_cast<int>(nodes.size());
    if (n < 2)
        return (n <= 1); // 0 or 1 vertex: trivially biconnected by convention

    // Map NodeId -> sequential index [0..n-1]
    std::unordered_map<NodeId, int> nodeIndex;
    nodeIndex.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i)
        nodeIndex[nodes[static_cast<std::size_t>(i)]] = i;

    std::vector<int> dfsNum(static_cast<std::size_t>(n), 0);
    std::vector<int> low(static_cast<std::size_t>(n), 0);
    std::vector<int> parent(static_cast<std::size_t>(n), -1);
    int counter = 0;
    bool foundArticulation = false;

    // Recursive DFS lambda
    auto dfs = [&](this auto&& self, int vi) -> void {
        auto u = static_cast<std::size_t>(vi);
        ++counter;
        dfsNum[u] = counter;
        low[u] = counter;
        int childCount = 0;

        for (const auto& neighbor : graph.getNeighbors(nodes[u])) {
            auto it = nodeIndex.find(neighbor);
            if (it == nodeIndex.end())
                continue;
            int wi = it->second;
            auto w = static_cast<std::size_t>(wi);

            if (dfsNum[w] == 0) {
                ++childCount;
                parent[w] = vi;
                self(wi);

                low[u] = std::min(low[u], low[w]);

                // Articulation point check
                if (parent[u] == -1 && childCount > 1) {
                    foundArticulation = true;
                    return;
                }
                if (parent[u] != -1 && low[w] >= dfsNum[u]) {
                    foundArticulation = true;
                    return;
                }
            } else if (wi != parent[u]) {
                low[u] = std::min(low[u], dfsNum[w]);
            }
        }
    };

    dfs(0);

    if (foundArticulation)
        return false;

    // Check all vertices were visited (connected)
    for (int i = 0; i < n; ++i) {
        if (dfsNum[static_cast<std::size_t>(i)] == 0)
            return false;
    }

    return true;
}

/**
 * @brief Find all cut vertices (articulation points) in a graph.
 *
 * A cut vertex is one whose removal disconnects the graph. Uses Tarjan's
 * DFS-based algorithm with lowpoint tracking. Handles disconnected graphs
 * by running DFS from each unvisited vertex.
 *
 * @tparam G A type satisfying the GraphBase concept.
 * @param graph The input graph to search.
 * @return A vector of node identifiers that are cut vertices. Returns an
 *         empty vector if none exist.
 *
 * @par Complexity
 * O(V + E)
 */
template <GraphBase G>
std::vector<typename G::NodeIdType> findCutVertices(const G& graph)
{
    using NodeId = typename G::NodeIdType;

    std::vector<NodeId> nodes;
    for (const auto& n : graph.getNodes())
        nodes.push_back(n);

    int n = static_cast<int>(nodes.size());
    if (n < 2)
        return {};

    std::unordered_map<NodeId, int> nodeIndex;
    nodeIndex.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i)
        nodeIndex[nodes[static_cast<std::size_t>(i)]] = i;

    std::vector<int> dfsNum(static_cast<std::size_t>(n), 0);
    std::vector<int> low(static_cast<std::size_t>(n), 0);
    std::vector<int> parent(static_cast<std::size_t>(n), -1);
    std::vector<bool> isCut(static_cast<std::size_t>(n), false);
    int counter = 0;

    auto dfs = [&](this auto&& self, int vi) -> void {
        auto u = static_cast<std::size_t>(vi);
        ++counter;
        dfsNum[u] = counter;
        low[u] = counter;
        int childCount = 0;

        for (const auto& neighbor : graph.getNeighbors(nodes[u])) {
            auto it = nodeIndex.find(neighbor);
            if (it == nodeIndex.end())
                continue;
            int wi = it->second;
            auto w = static_cast<std::size_t>(wi);

            if (dfsNum[w] == 0) {
                ++childCount;
                parent[w] = vi;
                self(wi);

                low[u] = std::min(low[u], low[w]);

                if (parent[u] == -1 && childCount > 1) {
                    isCut[u] = true;
                }
                if (parent[u] != -1 && low[w] >= dfsNum[u]) {
                    isCut[u] = true;
                }
            } else if (wi != parent[u]) {
                low[u] = std::min(low[u], dfsNum[w]);
            }
        }
    };

    // Run DFS from each unvisited vertex (handles disconnected graphs)
    for (int i = 0; i < n; ++i) {
        if (dfsNum[static_cast<std::size_t>(i)] == 0)
            dfs(i);
    }

    std::vector<NodeId> result;
    for (int i = 0; i < n; ++i) {
        if (isCut[static_cast<std::size_t>(i)])
            result.push_back(nodes[static_cast<std::size_t>(i)]);
    }
    return result;
}

} // namespace MathUtils

#endif // GRAPH_ALGORITHMS_HPP
