#ifndef SEPARATION_PAIRS_HPP
#define SEPARATION_PAIRS_HPP

#include <structures/graph.hpp>
#include <structures/graph_algorithms.hpp>

#include <algorithm>
#include <cassert>
#include <expected>
#include <numeric>
#include <ranges>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace MathUtils {

/**
 * @brief Classification of separation pairs.
 *
 * - Type1: Identified via the lowpoint-based condition in pathSearch.
 * - Type2: Identified via the TSTACK or degree-2 vertex condition.
 * - Type3: Caused by multiple edges between the same pair of vertices.
 */
enum class SeparationPairType { Type1, Type2, Type3 };

/**
 * @brief A separation pair in a graph.
 *
 * Represents two vertices @p a and @p b whose removal (along with their
 * incident edges) disconnects the graph into two or more components.
 *
 * @tparam NodeIdType The node identifier type of the graph.
 */
template <typename NodeIdType>
struct SeparationPair {
    NodeIdType a; ///< First vertex of the separation pair.
    NodeIdType b; ///< Second vertex of the separation pair.
    SeparationPairType type; ///< Classification of this separation pair.
};

/**
 * @brief Error codes returned by the separation-pairs algorithm.
 */
enum class SeparationPairsError {
    NotBiconnected, ///< The input graph is not biconnected.
    HasSelfLoops, ///< The input graph contains self-loops.
    NoPairsFound, ///< The graph has no separation pairs.
};

/// @cond INTERNAL
/// @brief Implementation details for the separation-pairs algorithm.
namespace detail {

    /**
     * @brief Internal edge representation used by the algorithm.
     *
     * After DFS classification the fields have the following semantics:
     * - **Tree arc**: @p source is the parent, @p target is the child.
     * - **Frond** (back edge): @p source is the descendant (higher DFS
     *   number), @p target is the ancestor (lower DFS number).
     */
    struct InternalEdge {
        int id {}; ///< Unique edge identifier.
        int source {}; ///< Source vertex (meaning depends on edge kind).
        int target {}; ///< Target vertex (meaning depends on edge kind).
        bool isTreeArc { false }; ///< @c true if this edge is a tree arc.
        bool startsPath { false }; ///< @c true if this edge begins a new path.
        bool classified {
            false
        }; ///< @c true once DFS has classified this edge.
    };

    /**
     * @brief Triple stored on the TSTACK during pathSearch.
     *
     * Each triple @c {h, a, b} represents a potential Type-2 separation
     * pair @c {a, b}, where @p h is the highest-numbered vertex in the
     * corresponding split component. The sentinel value @p isEOS marks an
     * end-of-stack boundary.
     */
    struct Triple {
        int h { -1 }; ///< Highest-numbered vertex in the split component.
        int a { -1 }; ///< First vertex of the potential separation pair.
        int b { -1 }; ///< Second vertex of the potential separation pair.
        bool isEOS { false }; ///< @c true if this is an end-of-stack marker.

        /** @brief Create an end-of-stack sentinel triple. */
        static Triple eos() { return { -1, -1, -1, true }; }
    };

    /**
     * @brief Encapsulates all state for one execution of the separation-pairs
     *        algorithm.
     *
     * Implements the Hopcroft-Tarjan (1973) algorithm with the corrections
     * by Gutwenger & Mutzel (2001). The algorithm runs in nine phases:
     * -# Map nodes and edges to an internal representation.
     * -# Detect multiple edges (Type-3 pairs).
     * -# Build the DFS palm tree.
     * -# Compute lowpoints.
     * -# Renumber vertices to satisfy properties (P1)-(P3).
     * -# Sort adjacency lists by the phi-function.
     * -# Mark path starts.
     * -# Initialise auxiliary structures.
     * -# PathSearch — detect Type-1 and Type-2 pairs.
     *
     * @tparam G A type satisfying the GraphBase concept.
     */
    template <GraphBase G>
    class SeparationPairsFinder {
    public:
        using NodeId = typename G::NodeIdType;
        using EdgeId = typename G::EdgeIdType;
        using Result = std::expected<std::vector<SeparationPair<NodeId>>,
            SeparationPairsError>;

        /**
         * @brief Construct a finder for the given graph.
         * @param graph The biconnected graph to analyse.
         * @param stopAfterFirst If @c true, the algorithm terminates as
         *        soon as the first separation pair is found.
         */
        explicit SeparationPairsFinder(
            const G& graph, bool stopAfterFirst = false)
            : m_graph(graph)
            , m_stopAfterFirst(stopAfterFirst)
        {
        }

        /**
         * @brief Execute the separation-pairs algorithm.
         *
         * Runs all nine phases of the algorithm and returns the detected
         * separation pairs, or an error if the preconditions are violated.
         *
         * @return On success, a vector of SeparationPair structs.
         * @return On failure, a SeparationPairsError indicating the reason.
         */
        Result run()
        {
            // Phase 0: Map nodes and edges to internal representation
            mapNodes();

            if (m_n < 2) {
                // Trivially no separation pairs
                return std::vector<SeparationPair<NodeId>> {};
            }

            if (auto err = checkSelfLoops(); err.has_value())
                return std::unexpected(*err);

            buildInternalGraph();

            // Phase 1: Detect multiple edges (Type-3 pairs)
            detectMultipleEdges();

            // Remove duplicate edges for the main algorithm
            removeDuplicateEdges();

            // Biconnectivity check on the simple graph
            if (!checkBiconnected())
                return std::unexpected(SeparationPairsError::NotBiconnected);

            // Trivial: fewer than 4 vertices means no separation pairs
            // (Type-3 may still have been found above)
            if (m_n < 4)
                return convertResults();

            // Early exit: a Type-3 pair was already found
            if (m_foundFirst)
                return convertResults();

            // Phase 2: Initial DFS — build palm tree
            initialDFS();

            // Phase 3: Compute lowpoints
            computeLowpoints();

            // Phase 4: Renumber vertices to satisfy (P1)-(P3)
            renumberVertices();

            // Phase 5: Sort adjacency lists by phi-function
            sortAdjacencyLists();

            // Phase 6: Mark path starts
            markPathStarts();

            // Phase 7: Initialize auxiliary structures
            initializeAuxiliary();

            // Phase 8: PathSearch — detect Type-1 and Type-2 pairs
            m_TSTACK.push(Triple::eos());
            pathSearch(1);

            // Phase 9: Convert results back to original NodeId
            return convertResults();
        }

    private:
        const G& m_graph; ///< Reference to the input graph.
        bool m_stopAfterFirst {
            false
        }; ///< If true, stop after the first pair is found.
        bool m_foundFirst {
            false
        }; ///< Set to true when the first pair is recorded.

        /// @name Node mapping
        /// @{
        int m_n {}; ///< Number of vertices.
        int m_m {}; ///< Number of edges (after deduplication).
        std::vector<NodeId> m_indexToNode; ///< Maps [1..n] to original NodeId.
        std::unordered_map<NodeId, int>
            m_nodeToIndex; ///< Maps original NodeId to [1..n].
        /// @}

        /// @name Edge storage
        /// @{
        std::vector<InternalEdge> m_edges; ///< All internal edges.
        /// @brief Adjacency lists; @c m_adj[v] holds edge indices into
        ///        @ref m_edges. Indexed [0..n], slots [1..n] used.
        std::vector<std::vector<int>> m_adj;
        /// @}

        /// @name DFS tree info (indexed [0..n], using [1..n])
        /// @{
        std::vector<int> m_parent; ///< DFS tree parent of each vertex.
        std::vector<int> m_dfsNum; ///< DFS discovery number.
        std::vector<int> m_ND; ///< Number of descendants (including self).
        std::vector<int> m_lowpt1; ///< First lowpoint.
        std::vector<int> m_lowpt2; ///< Second lowpoint.
        std::vector<int> m_high; ///< Highest-numbered frond target.
        std::vector<int> m_degree; ///< Vertex degree.
        /// @}

        /// @name Renumbering
        /// @{
        std::vector<int>
            m_newNumber; ///< Old internal index to new internal index.
        std::vector<int>
            m_oldNumber; ///< New internal index to old internal index.
        /// @}

        /// @name Auxiliary structures
        /// @{
        std::vector<int>
            m_visitedTreeArcs; ///< Count of visited tree arcs per vertex.
        std::vector<int> m_totalTreeArcs; ///< Total tree arcs per vertex.
        std::vector<std::vector<int>>
            m_frondList; ///< Frond edge ids ending at each vertex.
        /// @}

        /// @name Stacks
        /// @{
        std::stack<int> m_ESTACK; ///< Edge stack.
        std::stack<Triple> m_TSTACK; ///< Triple stack for Type-2 detection.
        /// @}

        /// @name Results
        /// @{

        /**
         * @brief An intermediate result stored using renumbered vertex indices.
         */
        struct InternalPair {
            int a; ///< First vertex (renumbered).
            int b; ///< Second vertex (renumbered).
            SeparationPairType type; ///< Pair classification.
        };
        std::vector<InternalPair>
            m_internalResults; ///< Type-1 / Type-2 results.

        /// @brief Type-3 results collected before renumbering (original
        /// NodeId).
        std::vector<SeparationPair<NodeId>> m_multipleEdgeResults;
        /// @}

        /**
         * @brief Phase 0: Assign sequential 1..n indices to graph nodes.
         */
        void mapNodes()
        {
            m_indexToNode.push_back(NodeId {}); // slot 0 unused (1-indexed)

            for (const auto& node : m_graph.getNodes()) {
                m_indexToNode.push_back(node);
            }

            m_n = static_cast<int>(m_indexToNode.size()) - 1;

            m_nodeToIndex.reserve(static_cast<std::size_t>(m_n));
            for (int i = 1; i <= m_n; ++i) {
                m_nodeToIndex[m_indexToNode[sz(i)]] = i;
            }
        }

        /**
         * @brief Check for self-loops in the input graph.
         * @return The error if a self-loop is found, @c std::nullopt otherwise.
         */
        std::optional<SeparationPairsError> checkSelfLoops()
        {
            for (const auto& edgeId : m_graph.getEdges()) {
                auto [s, t] = m_graph.getEndpoints(edgeId);
                if (s == t)
                    return SeparationPairsError::HasSelfLoops;
            }
            return std::nullopt;
        }

        /**
         * @brief Build the internal adjacency-list representation from the
         *        input graph.
         */
        void buildInternalGraph()
        {
            m_adj.resize(static_cast<std::size_t>(m_n + 1));

            int edgeCounter = 0;
            for (const auto& edgeId : m_graph.getEdges()) {
                auto [s, t] = m_graph.getEndpoints(edgeId);

                auto siIt = m_nodeToIndex.find(s);
                auto tiIt = m_nodeToIndex.find(t);
                assert(siIt != m_nodeToIndex.end()
                    && "buildInternalGraph: node not in index map");
                assert(tiIt != m_nodeToIndex.end()
                    && "buildInternalGraph: node not in index map");
                int si = siIt->second;
                int ti = tiIt->second;

                InternalEdge ie;
                ie.id = edgeCounter;
                ie.source = si;
                ie.target = ti;
                m_edges.push_back(ie);

                m_adj[sz(si)].push_back(edgeCounter);
                m_adj[sz(ti)].push_back(edgeCounter);

                ++edgeCounter;
            }

            m_m = edgeCounter;
        }

        /**
         * @brief Phase 1: Detect multiple edges between the same vertex pair
         *        and record them as Type-3 separation pairs.
         */
        void detectMultipleEdges()
        {
            if (m_m < 4)
                return; // Need >= 4 total edges for multiple edge separation

            // Count edges between each pair of vertices
            std::unordered_map<long long, int> edgeCounts;
            for (const auto& e : m_edges) {
                int lo = std::min(e.source, e.target);
                int hi = std::max(e.source, e.target);
                long long key = static_cast<long long>(lo) * (m_n + 1) + hi;
                edgeCounts[key]++;
            }

            for (const auto& [key, count] : edgeCounts) {
                if (count >= 2) {
                    int hi = static_cast<int>(key % (m_n + 1));
                    int lo = static_cast<int>(key / (m_n + 1));
                    m_multipleEdgeResults.push_back({ m_indexToNode[sz(lo)],
                        m_indexToNode[sz(hi)], SeparationPairType::Type3 });

                    if (m_stopAfterFirst) {
                        m_foundFirst = true;
                        return;
                    }
                }
            }
        }

        /**
         * @brief Remove duplicate edges, keeping only one edge per vertex pair.
         *
         * Rebuilds the adjacency lists with unique edges only. Called after
         * Type-3 detection so the main algorithm operates on a simple graph.
         */
        void removeDuplicateEdges()
        {
            std::unordered_set<long long> seen;
            std::vector<InternalEdge> uniqueEdges;

            // Rebuild adjacency
            for (int i = 0; i <= m_n; ++i)
                m_adj[sz(i)].clear();

            int newId = 0;
            for (const auto& e : m_edges) {
                int lo = std::min(e.source, e.target);
                int hi = std::max(e.source, e.target);
                long long key = static_cast<long long>(lo) * (m_n + 1) + hi;

                if (seen.insert(key).second) {
                    InternalEdge ne = e;
                    ne.id = newId;
                    uniqueEdges.push_back(ne);
                    m_adj[sz(ne.source)].push_back(newId);
                    m_adj[sz(ne.target)].push_back(newId);
                    ++newId;
                }
            }

            m_edges = std::move(uniqueEdges);
            m_m = static_cast<int>(m_edges.size());
        }

        /**
         * @brief Check biconnectivity on the internal simple graph.
         *
         * Uses Tarjan's algorithm. Must be called after duplicate edges have
         * been removed.
         *
         * @return @c true if the graph is biconnected.
         */
        bool checkBiconnected()
        {
            if (m_n < 2)
                return true;

            // Tarjan's biconnectivity on internal graph
            std::vector<int> disc(sz(m_n + 1), 0);
            std::vector<int> low(sz(m_n + 1), 0);
            std::vector<int> par(sz(m_n + 1), -1);
            int counter = 0;
            bool foundArt = false;

            auto dfs = [&](this auto&& self, int v) -> void {
                ++counter;
                disc[sz(v)] = counter;
                low[sz(v)] = counter;
                int childCount = 0;

                for (int eidx : m_adj[sz(v)]) {
                    const auto& edge = m_edges[sz(eidx)];
                    int w = (edge.source == v) ? edge.target : edge.source;

                    if (disc[sz(w)] == 0) {
                        ++childCount;
                        par[sz(w)] = v;
                        self(w);
                        low[sz(v)] = std::min(low[sz(v)], low[sz(w)]);

                        if (par[sz(v)] == -1 && childCount > 1)
                            foundArt = true;
                        if (par[sz(v)] != -1 && low[sz(w)] >= disc[sz(v)])
                            foundArt = true;
                    } else if (w != par[sz(v)]) {
                        low[sz(v)] = std::min(low[sz(v)], disc[sz(w)]);
                    }
                }
            };

            dfs(1);

            if (foundArt)
                return false;

            // Check connectivity
            for (int i = 1; i <= m_n; ++i) {
                if (disc[sz(i)] == 0)
                    return false;
            }
            return true;
        }

        /**
         * @brief Phase 2: Perform the initial DFS to build the palm tree.
         *
         * Classifies every edge as either a tree arc or a frond (back edge)
         * and computes parent pointers and descendant counts.
         */
        void initialDFS()
        {
            m_parent.assign(sz(m_n + 1), 0);
            m_dfsNum.assign(sz(m_n + 1), 0);
            m_ND.assign(sz(m_n + 1), 0);

            int dfsCounter = 0;

            // Mark all edges as unclassified
            for (auto& e : m_edges)
                e.classified = false;

            auto dfs = [&](this auto&& self, int v, int p) -> void {
                ++dfsCounter;
                m_dfsNum[sz(v)] = dfsCounter;
                m_parent[sz(v)] = p;
                m_ND[sz(v)] = 1;

                for (int eidx : m_adj[sz(v)]) {
                    auto& edge = m_edges[sz(eidx)];

                    if (edge.classified)
                        continue;

                    int w = (edge.source == v) ? edge.target : edge.source;

                    if (m_dfsNum[sz(w)] == 0) {
                        // Tree arc: v -> w
                        edge.classified = true;
                        edge.isTreeArc = true;
                        edge.source = v;
                        edge.target = w;

                        self(w, v);
                        m_ND[sz(v)] += m_ND[sz(w)];

                    } else if (m_dfsNum[sz(w)] < m_dfsNum[sz(v)]) {
                        // Frond: v ~> w (back edge to ancestor)
                        edge.classified = true;
                        edge.isTreeArc = false;
                        edge.source = v; // descendant (higher DFS#)
                        edge.target = w; // ancestor (lower DFS#)
                    }
                    // If dfsNum[w] > dfsNum[v], this edge was already
                    // classified as a tree arc from the other direction.
                }
            };

            dfs(1, 0);
        }

        /**
         * @brief Phase 3: Compute the first and second lowpoints for every
         *        vertex.
         *
         * Processes vertices in post-order so that children are resolved
         * before their parents.
         */
        void computeLowpoints()
        {
            m_lowpt1.assign(sz(m_n + 1), 0);
            m_lowpt2.assign(sz(m_n + 1), 0);

            // Process in post-order (reverse DFS finish order)
            std::vector<int> postOrder;
            postOrder.reserve(sz(m_n));
            {
                std::vector<bool> visited(sz(m_n + 1), false);

                auto postDFS = [&](this auto&& self, int v) -> void {
                    visited[sz(v)] = true;
                    for (int eidx : m_adj[sz(v)]) {
                        const auto& edge = m_edges[sz(eidx)];
                        if (edge.isTreeArc && edge.source == v
                            && !visited[sz(edge.target)]) {
                            self(edge.target);
                        }
                    }
                    postOrder.push_back(v);
                };

                postDFS(1);
            }

            for (int v : postOrder) {
                m_lowpt1[sz(v)] = m_dfsNum[sz(v)];
                m_lowpt2[sz(v)] = m_dfsNum[sz(v)];

                for (int eidx : m_adj[sz(v)]) {
                    const auto& edge = m_edges[sz(eidx)];

                    if (edge.isTreeArc && edge.source == v) {
                        // Tree arc to child w
                        int w = edge.target;
                        updateLowpoints(v, m_lowpt1[sz(w)]);

                        // Also consider child's lowpt2
                        if (m_lowpt2[sz(w)] < m_lowpt2[sz(v)]) {
                            if (m_lowpt2[sz(w)] != m_lowpt1[sz(v)]) {
                                m_lowpt2[sz(v)] = m_lowpt2[sz(w)];
                            } else if (m_lowpt1[sz(w)] < m_lowpt2[sz(v)]) {
                                // lowpt2[w] == lowpt1[v], so use lowpt1[w]
                                // as candidate for lowpt2[v]
                                m_lowpt2[sz(v)] = m_lowpt1[sz(w)];
                            }
                        }

                    } else if (!edge.isTreeArc
                        && m_dfsNum[sz(edge.target)]
                            < m_dfsNum[sz(edge.source)]) {
                        // Frond: source ~> target (source has higher DFS#)
                        if (edge.source == v) {
                            updateLowpoints(v, m_dfsNum[sz(edge.target)]);
                        }
                    }
                }
            }
        }

        /**
         * @brief Update the lowpoints of vertex @p v with a new @p candidate
         *        value.
         * @param v The vertex whose lowpoints may be updated.
         * @param candidate The candidate DFS number to consider.
         */
        void updateLowpoints(int v, int candidate)
        {
            if (candidate < m_lowpt1[sz(v)]) {
                m_lowpt2[sz(v)] = m_lowpt1[sz(v)];
                m_lowpt1[sz(v)] = candidate;
            } else if (candidate != m_lowpt1[sz(v)]
                && candidate < m_lowpt2[sz(v)]) {
                m_lowpt2[sz(v)] = candidate;
            }
        }

        /**
         * @brief Phase 4: Renumber vertices so that properties (P1)-(P3) hold.
         *
         * Sorts children by the phi-function, assigns new contiguous DFS
         * numbers, and remaps all internal arrays accordingly.
         */
        void renumberVertices()
        {
            m_newNumber.assign(sz(m_n + 1), 0);
            m_oldNumber.assign(sz(m_n + 1), 0);

            // First, sort children of each vertex by the phi-function
            // to get the correct tree-child ordering before assigning numbers.
            // Children order determines the contiguous ranges.
            sortChildrenByPhi();

            int counter = 0;
            assignNumbers(1, counter);

            applyNewNumbering();
        }

        /**
         * @brief Sort tree-arc children of each vertex by the phi-function.
         *
         * The resulting child order is used by assignNumbers() to satisfy
         * property (P2).
         */
        void sortChildrenByPhi()
        {
            for (int v = 1; v <= m_n; ++v) {
                // Collect outgoing tree arcs from v and sort by phi
                std::ranges::sort(m_adj[sz(v)], [&](int a, int b) {
                    return computePhiPreRenumber(v, a)
                        < computePhiPreRenumber(v, b);
                });
            }
        }

        /**
         * @brief Compute the phi-function using original (pre-renumber) DFS
         *        numbers.
         *
         * Used only for sorting children before the renumbering step.
         *
         * @param v The vertex whose adjacency list is being sorted.
         * @param edgeIdx Index into @ref m_edges.
         * @return The phi value for the edge.
         */
        int computePhiPreRenumber(int v, int edgeIdx)
        {
            const auto& edge = m_edges[sz(edgeIdx)];
            constexpr int INF = 1000000000;

            if (edge.isTreeArc) {
                // Only consider outgoing tree arcs from v
                if (edge.source != v)
                    return INF;

                int w = edge.target;
                if (m_lowpt2[sz(w)] < m_dfsNum[sz(v)]) {
                    return 3 * m_lowpt1[sz(w)];
                } else {
                    return 3 * m_lowpt1[sz(w)] + 2;
                }
            } else {
                // Frond: only consider outgoing fronds from v
                if (edge.source != v)
                    return INF;

                int w = edge.target;
                return 3 * m_dfsNum[sz(w)] + 1;
            }
        }

        /**
         * @brief Recursively assign new DFS numbers in pre-order.
         * @param v Current vertex (old numbering).
         * @param counter Running counter for the new numbering.
         */
        void assignNumbers(int v, int& counter)
        {
            ++counter;
            m_newNumber[sz(v)] = counter;
            m_oldNumber[sz(counter)] = v;

            // Visit children in adjacency list order (already sorted)
            for (int eidx : m_adj[sz(v)]) {
                const auto& edge = m_edges[sz(eidx)];
                if (edge.isTreeArc && edge.source == v) {
                    assignNumbers(edge.target, counter);
                }
            }
        }

        /**
         * @brief Remap all internal arrays (parent, dfsNum, lowpt, ND, edges,
         *        adjacency, node maps) to the new vertex numbering.
         */
        void applyNewNumbering()
        {
            std::vector<int> oldParent = m_parent;
            std::vector<int> oldND = m_ND;
            std::vector<int> oldLowpt1 = m_lowpt1;
            std::vector<int> oldLowpt2 = m_lowpt2;
            std::vector<int> oldDfsNum = m_dfsNum;

            // Update parent
            m_parent.assign(sz(m_n + 1), 0);
            for (int v = 1; v <= m_n; ++v) {
                if (oldParent[sz(v)] != 0) {
                    m_parent[sz(m_newNumber[sz(v)])]
                        = m_newNumber[sz(oldParent[sz(v)])];
                } else {
                    m_parent[sz(m_newNumber[sz(v)])] = 0;
                }
            }

            // Update dfsNum — after renumbering, dfsNum[v] == v
            m_dfsNum.assign(sz(m_n + 1), 0);
            for (int v = 1; v <= m_n; ++v)
                m_dfsNum[sz(v)] = v;

            // Update lowpt1/lowpt2: these are DFS numbers (vertex numbers),
            // so remap through the old->new mapping.
            m_lowpt1.assign(sz(m_n + 1), 0);
            m_lowpt2.assign(sz(m_n + 1), 0);
            for (int v = 1; v <= m_n; ++v) {
                int newV = m_newNumber[sz(v)];
                // lowpt1[v] was a DFS number in old numbering.
                // Find which old vertex had that DFS number.
                // In old numbering, dfsNum[u] == oldDfsNum[u], and
                // lowpt1[v] referred to the DFS number of the lowpoint vertex.
                // We need to find the old vertex u with oldDfsNum[u] ==
                // oldLowpt1[v], then newNumber[u] gives the new DFS number.
                //
                // Since in the initial DFS numbering dfsNum corresponds to
                // discovery order (which is what lowpt stores), we need
                // an inverse map from old DFS number -> old vertex.
                //
                // However, there's a simpler approach: lowpt1[v] is the DFS
                // number of some vertex u. We stored oldDfsNum, and before
                // renumbering dfsNum[u] == oldDfsNum[u]. So we need
                // oldDfsNumInverse[oldLowpt1[v]] to get u, then newNumber[u].
                //
                // But actually we already know the identity: in the original
                // DFS, vertices were numbered 1..n as internal indices, and
                // dfsNum[v] was the discovery number. The lowpoints store
                // DFS numbers, not vertex indices. We need to convert.
                //
                // Build inverse once below.
                (void)newV; // handled in batch below
            }

            // Build inverse: oldDfsNum -> old vertex
            std::vector<int> dfsNumToOldVertex(sz(m_n + 1), 0);
            for (int v = 1; v <= m_n; ++v) {
                dfsNumToOldVertex[sz(oldDfsNum[sz(v)])] = v;
            }

            for (int v = 1; v <= m_n; ++v) {
                int newV = m_newNumber[sz(v)];

                // lowpt1[v] is an old DFS number. Find vertex with that number.
                int oldLp1Vertex = dfsNumToOldVertex[sz(oldLowpt1[sz(v)])];
                m_lowpt1[sz(newV)] = m_newNumber[sz(oldLp1Vertex)];

                int oldLp2Vertex = dfsNumToOldVertex[sz(oldLowpt2[sz(v)])];
                m_lowpt2[sz(newV)] = m_newNumber[sz(oldLp2Vertex)];
            }

            // Update ND
            m_ND.assign(sz(m_n + 1), 0);
            for (int v = 1; v <= m_n; ++v) {
                m_ND[sz(m_newNumber[sz(v)])] = oldND[sz(v)];
            }

            // Update edge endpoints
            for (auto& edge : m_edges) {
                edge.source = m_newNumber[sz(edge.source)];
                edge.target = m_newNumber[sz(edge.target)];
            }

            // Update indexToNode mapping
            std::vector<NodeId> newIndexToNode(sz(m_n + 1));
            for (int newV = 1; newV <= m_n; ++newV) {
                int oldV = m_oldNumber[sz(newV)];
                newIndexToNode[sz(newV)] = m_indexToNode[sz(oldV)];
            }
            m_indexToNode = std::move(newIndexToNode);

            // Update nodeToIndex
            m_nodeToIndex.clear();
            for (int i = 1; i <= m_n; ++i) {
                m_nodeToIndex[m_indexToNode[sz(i)]] = i;
            }

            // Rebuild adjacency lists
            for (int i = 0; i <= m_n; ++i)
                m_adj[sz(i)].clear();

            for (int i = 0; i < static_cast<int>(m_edges.size()); ++i) {
                const auto& edge = m_edges[sz(i)];
                m_adj[sz(edge.source)].push_back(i);
                m_adj[sz(edge.target)].push_back(i);
            }
        }

        /**
         * @brief Phase 5: Sort every vertex's adjacency list by the
         *        (post-renumbering) phi-function.
         */
        void sortAdjacencyLists()
        {
            // After renumbering, dfsNum[v] == v, so phi uses vertex
            // numbers directly.
            for (int v = 1; v <= m_n; ++v) {
                std::ranges::sort(m_adj[sz(v)], [&](int a, int b) {
                    return computePhi(v, a) < computePhi(v, b);
                });
            }
        }

        /**
         * @brief Corrected Gutwenger-Mutzel phi-function (post-renumbering).
         *
         * At this point @c dfsNum[v] == v for all vertices.
         *
         * @param v The vertex whose adjacency list is being sorted.
         * @param edgeIdx Index into @ref m_edges.
         * @return The phi value for the edge.
         */
        int computePhi(int v, int edgeIdx)
        {
            const auto& edge = m_edges[sz(edgeIdx)];
            constexpr int INF = 1000000000;

            if (edge.isTreeArc) {
                // Outgoing tree arc: v is parent of w
                int w;
                if (edge.source == v && m_parent[sz(edge.target)] == v) {
                    w = edge.target;
                } else if (edge.target == v && m_parent[sz(edge.source)] == v) {
                    w = edge.source;
                } else {
                    return INF; // Not an outgoing tree arc from v
                }

                if (m_lowpt2[sz(w)] < v) {
                    return 3 * m_lowpt1[sz(w)];
                } else {
                    return 3 * m_lowpt1[sz(w)] + 2;
                }
            } else {
                // Frond: outgoing means v -> ancestor (ancestor has lower
                // number)
                int w;
                if (edge.source == v && edge.target < v) {
                    w = edge.target;
                } else if (edge.target == v && edge.source < v) {
                    w = edge.source;
                } else {
                    return INF; // Not an outgoing frond from v
                }
                return 3 * w + 1;
            }
        }

        /**
         * @brief Phase 6: Mark edges that begin a new path in the path
         *        decomposition.
         */
        void markPathStarts()
        {
            for (int v = 1; v <= m_n; ++v) {
                bool previousWasFrond = true; // First edge always starts a path

                for (int eidx : m_adj[sz(v)]) {
                    auto& edge = m_edges[sz(eidx)];

                    if (!isOutgoingEdge(v, edge))
                        continue;

                    edge.startsPath = previousWasFrond;

                    if (edge.isTreeArc) {
                        previousWasFrond = false;
                    } else {
                        previousWasFrond = true;
                    }
                }
            }
        }

        /**
         * @brief Test whether @p edge is an outgoing edge from vertex @p v.
         * @param v The vertex to test from.
         * @param edge The edge to classify.
         * @return @c true if the edge leaves @p v (tree arc or frond to
         *         ancestor).
         */
        static bool isOutgoingEdge(int v, const InternalEdge& edge)
        {
            if (edge.isTreeArc) {
                // Outgoing tree arc: source == v
                return edge.source == v;
            } else {
                // Outgoing frond: source == v and target < v (to ancestor)
                return edge.source == v && edge.target < v;
            }
        }

        /**
         * @brief Phase 7: Initialise degree counts, tree-arc counters, frond
         *        lists, and high-point values.
         */
        void initializeAuxiliary()
        {
            // Degree
            m_degree.assign(sz(m_n + 1), 0);
            for (int v = 1; v <= m_n; ++v)
                m_degree[sz(v)] = static_cast<int>(m_adj[sz(v)].size());

            // Tree arc counters
            m_visitedTreeArcs.assign(sz(m_n + 1), 0);
            m_totalTreeArcs.assign(sz(m_n + 1), 0);
            for (int v = 1; v <= m_n; ++v) {
                int count = 0;
                for (int eidx : m_adj[sz(v)]) {
                    const auto& edge = m_edges[sz(eidx)];
                    if (edge.isTreeArc && edge.source == v)
                        ++count;
                }
                m_totalTreeArcs[sz(v)] = count;
            }

            // Frond lists and high values
            m_frondList.assign(sz(m_n + 1), {});
            m_high.assign(sz(m_n + 1), 0);

            // Collect fronds in DFS visit order
            collectFrondsInOrder(1);

            // Set initial high values
            for (int v = 1; v <= m_n; ++v) {
                if (!m_frondList[sz(v)].empty()) {
                    int firstFrondIdx = m_frondList[sz(v)].front();
                    m_high[sz(v)] = m_edges[sz(firstFrondIdx)].source;
                }
            }
        }

        /**
         * @brief Collect frond edges in DFS visit order and populate
         *        @ref m_frondList.
         * @param v The vertex to start the DFS traversal from.
         */
        void collectFrondsInOrder(int v)
        {
            for (int eidx : m_adj[sz(v)]) {
                const auto& edge = m_edges[sz(eidx)];

                if (edge.isTreeArc && edge.source == v) {
                    collectFrondsInOrder(edge.target);
                } else if (!edge.isTreeArc && edge.source == v) {
                    // Frond from v to edge.target (ancestor)
                    m_frondList[sz(edge.target)].push_back(eidx);
                }
            }
        }

        /**
         * @brief Phase 8: PathSearch — the main detection loop.
         *
         * Walks the adjacency lists in sorted order, maintaining the ESTACK
         * and TSTACK, and checks for Type-1 and Type-2 separation pairs
         * after each tree-arc subtree is fully explored.
         *
         * @param v The current vertex to process.
         */
        void pathSearch(int v)
        {
            for (int eidx : m_adj[sz(v)]) {
                if (m_foundFirst)
                    return;

                auto& edge = m_edges[sz(eidx)];

                if (edge.isTreeArc && edge.source == v) {
                    // ────── TREE ARC CASE: v -> w ──────
                    int w = edge.target;

                    if (edge.startsPath) {
                        updateTStackForTreeArc(v, w);
                        m_TSTACK.push(Triple::eos());
                    }

                    // Recurse into subtree
                    pathSearch(w);

                    // Mark this tree arc as visited
                    m_visitedTreeArcs[sz(v)]++;

                    // Push edge onto ESTACK
                    m_ESTACK.push(eidx);

                    // Check for separation pairs
                    checkType2Pairs(v, w);
                    checkType1Pair(v, w);

                    // Clean up TSTACK after path ends
                    if (edge.startsPath) {
                        // Remove all triples down to and including EOS
                        while (!m_TSTACK.empty() && !m_TSTACK.top().isEOS) {
                            m_TSTACK.pop();
                        }
                        if (!m_TSTACK.empty()) {
                            m_TSTACK.pop(); // Remove EOS marker
                        }

                        // Remove invalid triples based on high value
                        while (!m_TSTACK.empty() && !m_TSTACK.top().isEOS) {
                            auto [h, a, b, eos] = m_TSTACK.top();
                            if (a != v && b != v && m_high[sz(v)] > h) {
                                m_TSTACK.pop();
                            } else {
                                break;
                            }
                        }
                    }

                } else if (!edge.isTreeArc && edge.source == v) {
                    // ────── FROND CASE: v ~> w ──────
                    [[maybe_unused]] int w = edge.target;

                    if (edge.startsPath) {
                        updateTStackForFrond(v, edge.target);
                    }

                    // Push frond onto edge stack
                    m_ESTACK.push(eidx);
                }
            }
        }

        /**
         * @brief Update the TSTACK when a new tree-arc path starts at edge
         *        @p v -> @p w.
         * @param v Parent vertex.
         * @param w Child vertex.
         */
        void updateTStackForTreeArc(int v, int w)
        {
            int maxH = 0;
            int lastB = v;
            bool deletedAny = false;

            while (!m_TSTACK.empty() && !m_TSTACK.top().isEOS) {
                auto [h, a, b, eos] = m_TSTACK.top();
                if (a > m_lowpt1[sz(w)]) {
                    m_TSTACK.pop();
                    maxH = std::max(maxH, h);
                    lastB = b;
                    deletedAny = true;
                } else {
                    break;
                }
            }

            // Compute h for new triple: highest vertex in subtree of w
            int newH = w + m_ND[sz(w)] - 1;

            if (!deletedAny) {
                m_TSTACK.push({ newH, m_lowpt1[sz(w)], v, false });
            } else {
                m_TSTACK.push(
                    { std::max(maxH, newH), m_lowpt1[sz(w)], lastB, false });
            }
        }

        /**
         * @brief Update the TSTACK when a new frond path starts at edge
         *        @p v ~> @p w.
         * @param v Descendant vertex (higher DFS number).
         * @param w Ancestor vertex (lower DFS number).
         */
        void updateTStackForFrond(int v, int w)
        {
            int maxH = 0;
            int lastB = v;
            bool deletedAny = false;

            while (!m_TSTACK.empty() && !m_TSTACK.top().isEOS) {
                auto [h, a, b, eos] = m_TSTACK.top();
                if (a > w) {
                    m_TSTACK.pop();
                    maxH = std::max(maxH, h);
                    lastB = b;
                    deletedAny = true;
                } else {
                    break;
                }
            }

            if (!deletedAny) {
                m_TSTACK.push({ v, w, v, false });
            } else {
                m_TSTACK.push({ maxH, w, lastB, false });
            }
        }

        /**
         * @brief Check whether @c {lowpt1[w], v} is a Type-1 separation pair.
         *
         * The conditions are:
         * -# @c lowpt2[w] >= v
         * -# @c lowpt1[w] < v (subtree of @p w can reach below @p v)
         * -# There exists a vertex outside @p w's subtree
         *    (@c parent[v] != 1, or @p v has unvisited tree arcs).
         *
         * @param v Parent vertex.
         * @param w Child vertex.
         */
        void checkType1Pair(int v, int w)
        {

            if (m_lowpt2[sz(w)] >= v && m_lowpt1[sz(w)] < v) {
                bool hasOutsideVertex = (m_parent[sz(v)] != 1)
                    || (m_visitedTreeArcs[sz(v)] < m_totalTreeArcs[sz(v)]);

                if (hasOutsideVertex) {
                    int a = m_lowpt1[sz(w)];
                    int b = v;
                    addInternalResult(a, b, SeparationPairType::Type1);
                }
            }
        }

        /**
         * @brief Check for Type-2 separation pairs involving vertex @p v
         *        after returning from the subtree rooted at @p w.
         *
         * Iteratively examines the TSTACK and degree-2 vertex conditions.
         *
         * @param v Parent vertex.
         * @param w Child vertex from which we just returned.
         */
        void checkType2Pairs(int v, int w)
        {
            int currentW = w;

            while (v != 1) {
                bool foundPair = false;
                bool isTStackPair = false;

                // Check 1: TSTACK has triple with a == v
                if (!m_TSTACK.empty() && !m_TSTACK.top().isEOS) {
                    auto [h, a, b, eos] = m_TSTACK.top();

                    if (a == v) {
                        if (m_parent[sz(b)] == a) {
                            // False positive: b is direct child of a
                            m_TSTACK.pop();
                            continue;
                        } else {
                            foundPair = true;
                            isTStackPair = true;
                        }
                    }
                }

                // Check 2: Degree-2 vertex condition
                if (!foundPair) {
                    if (m_degree[sz(currentW)] == 2
                        && getFirstChild(currentW) > currentW) {
                        foundPair = true;
                        isTStackPair = false;
                    }
                }

                if (!foundPair)
                    break;

                if (!isTStackPair) {
                    // Degree-2 case
                    int a = v;
                    int b = getOtherNeighbor(currentW, v);

                    addInternalResult(std::min(a, b), std::max(a, b),
                        SeparationPairType::Type2);

                    currentW = b;
                } else {
                    // TSTACK pair
                    auto [h, a, b, eos] = m_TSTACK.top();
                    m_TSTACK.pop();

                    addInternalResult(a, b, SeparationPairType::Type2);

                    currentW = b;
                }
            }
        }

        /**
         * @brief Return the first tree-arc child of vertex @p v.
         * @param v The vertex to query.
         * @return The child's vertex number, or @c m_n+1 if @p v is a leaf.
         */
        int getFirstChild(int v)
        {
            for (int eidx : m_adj[sz(v)]) {
                const auto& edge = m_edges[sz(eidx)];
                if (edge.isTreeArc && edge.source == v)
                    return edge.target;
            }
            return m_n + 1; // No child (infinity)
        }

        /**
         * @brief Return a neighbour of @p v that is not @p exclude.
         *
         * Intended for degree-2 vertices where exactly one other neighbour
         * exists.
         *
         * @param v The vertex to query.
         * @param exclude The neighbour to skip.
         * @return The other neighbour, or @c -1 if none found (should not
         *         happen for degree-2 vertices).
         */
        int getOtherNeighbor(int v, int exclude)
        {
            for (int eidx : m_adj[sz(v)]) {
                const auto& edge = m_edges[sz(eidx)];
                int w = (edge.source == v) ? edge.target : edge.source;
                if (w != exclude)
                    return w;
            }
            return -1; // Should not happen for degree-2
        }

        /**
         * @brief Record a separation pair using renumbered vertex indices.
         * @param a First vertex (renumbered).
         * @param b Second vertex (renumbered).
         * @param type The classification of the pair.
         */
        void addInternalResult(int a, int b, SeparationPairType type)
        {
            int lo = std::min(a, b);
            int hi = std::max(a, b);
            m_internalResults.push_back({ lo, hi, type });

            if (m_stopAfterFirst)
                m_foundFirst = true;
        }

        /**
         * @brief Phase 9: Convert internal results back to original NodeId
         *        values and deduplicate.
         * @return The final vector of SeparationPair structs.
         */
        std::vector<SeparationPair<NodeId>> convertResults()
        {
            // Deduplicate: same (a,b) pair should appear only once,
            // but we keep the first type found.
            std::unordered_set<long long> seen;

            std::vector<SeparationPair<NodeId>> results;

            // Add Type-3 results first (these use original NodeIds already)
            for (const auto& sp : m_multipleEdgeResults) {
                // Create a canonical key for dedup (using internal indices)
                auto itA = m_nodeToIndex.find(sp.a);
                auto itB = m_nodeToIndex.find(sp.b);
                if (itA != m_nodeToIndex.end() && itB != m_nodeToIndex.end()) {
                    int lo = std::min(itA->second, itB->second);
                    int hi = std::max(itA->second, itB->second);
                    long long key = static_cast<long long>(lo) * (m_n + 1) + hi;
                    if (seen.insert(key).second) {
                        results.push_back(sp);
                    }
                }
            }

            // Add Type-1 and Type-2 results (convert from internal indices)
            for (const auto& ip : m_internalResults) {
                long long key = static_cast<long long>(ip.a) * (m_n + 1) + ip.b;
                if (seen.insert(key).second) {
                    results.push_back({ m_indexToNode[sz(ip.a)],
                        m_indexToNode[sz(ip.b)], ip.type });
                }
            }

            return results;
        }

        /** @brief Safe int-to-size_t cast for container indexing. */
        static std::size_t sz(int i) { return static_cast<std::size_t>(i); }
    };

} // namespace detail
/// @endcond

/**
 * @brief Find all separation pairs in a biconnected graph.
 *
 * Implements the Hopcroft-Tarjan (1973) separation-pairs algorithm with the
 * corrections by Gutwenger & Mutzel (2001, "A Linear Time Implementation of
 * SPQR-Trees").
 *
 * @tparam G A type satisfying the GraphBase concept.
 * @param graph The input graph (must be biconnected and free of self-loops).
 * @return On success, a vector of SeparationPair structs describing every
 *         separation pair found. On failure, a SeparationPairsError
 *         indicating the violated precondition.
 *
 * @par Complexity
 * O(V + E) time and space.
 */
template <GraphBase G>
std::expected<std::vector<SeparationPair<typename G::NodeIdType>>,
    SeparationPairsError>
findSeparationPairs(const G& graph)
{
    detail::SeparationPairsFinder<G> finder(graph);
    return finder.run();
}

/**
 * @brief Find the first separation pair in a biconnected graph.
 *
 * Uses the same Hopcroft-Tarjan / Gutwenger-Mutzel algorithm as
 * @ref findSeparationPairs, but terminates as soon as the first
 * separation pair is detected.
 *
 * @tparam G A type satisfying the GraphBase concept.
 * @param graph The input graph (must be biconnected and free of self-loops).
 * @return On success, the first SeparationPair found. On failure, a
 *         SeparationPairsError indicating the reason (including
 *         @c NoPairsFound when the graph has no separation pairs).
 *
 * @par Complexity
 * O(V + E) time and space (may terminate earlier in practice).
 */
template <GraphBase G>
std::expected<SeparationPair<typename G::NodeIdType>, SeparationPairsError>
findFirstSeparationPair(const G& graph)
{
    detail::SeparationPairsFinder<G> finder(graph, true);
    auto result = finder.run();

    if (!result.has_value())
        return std::unexpected(result.error());

    if (result->empty())
        return std::unexpected(SeparationPairsError::NoPairsFound);

    return result->front();
}

/**
 * @brief Check whether a graph is triconnected.
 *
 * A biconnected graph is triconnected if and only if it has no separation
 * pairs.
 *
 * @tparam G A type satisfying the GraphBase concept.
 * @param graph The input graph (must be biconnected and free of self-loops).
 * @return @c true if the graph is triconnected, or a SeparationPairsError
 *         if the preconditions are violated.
 *
 * @par Complexity
 * O(V + E) time and space.
 */
template <GraphBase G>
std::expected<bool, SeparationPairsError> isTriconnected(const G& graph)
{
    auto result = findSeparationPairs(graph);
    if (!result.has_value())
        return std::unexpected(result.error());
    return result->empty();
}

} // namespace MathUtils

#endif // SEPARATION_PAIRS_HPP
