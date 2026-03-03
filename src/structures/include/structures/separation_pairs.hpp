#ifndef SEPARATION_PAIRS_HPP
#define SEPARATION_PAIRS_HPP

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/Graph_d.h>
#include <ogdf/basic/simple_graph_alg.h>
#include <ogdf/graphalg/Triconnectivity.h> // Required for isTriconnected

#include <expected>
#include <optional>
#include <structures/graph.hpp>
#include <unordered_map>

namespace MathUtils {

template <typename NodeIdType>
struct SeparationPair {
    NodeIdType a;
    NodeIdType b;
};

enum class SeparationPairsError {
    NotBiconnected,
    HasSelfLoops,
};

namespace detail {

    template <typename G>
        requires GraphBase<G>
    auto findSeparationPair(const G& input)
        -> std::expected<std::optional<SeparationPair<typename G::NodeIdType>>,
            SeparationPairsError>
    {
        using NodeId = typename G::NodeIdType;
        ogdf::Graph g;

        std::unordered_map<NodeId, ogdf::node> forwardMap;
        // Map using the node index (int) to avoid hashing raw OGDF pointers
        std::unordered_map<int, NodeId> reverseMap;

        forwardMap.reserve(input.nodeCount());

        for (const auto& nid : input.getNodes()) {
            ogdf::node n = g.newNode();
            forwardMap[nid] = n;
            reverseMap[n->index()] = nid;
        }

        for (const auto& eid : input.getEdges()) {
            auto [uId, vId] = input.getEndpoints(eid);
            if (uId == vId)
                return std::unexpected(SeparationPairsError::HasSelfLoops);

            // at() is safer here to ensure the graph IDs are valid
            g.newEdge(forwardMap.at(uId), forwardMap.at(vId));
        }

        // Pre-condition Validation
        if (!ogdf::isBiconnected(g)) {
            return std::unexpected(SeparationPairsError::NotBiconnected);
        }

        ogdf::node s1 = nullptr;
        ogdf::node s2 = nullptr;

        // OGDF: returns true if triconnected, false if a pair exists
        bool triconnected = ogdf::isTriconnected(g, s1, s2);

        if (triconnected) {
            return std::nullopt;
        }

        if (s1 && s2) {
            return std::optional { SeparationPair<NodeId> {
                reverseMap.at(s1->index()), reverseMap.at(s2->index()) } };
        }

        return std::nullopt;
    }
} // namespace detail

template <GraphBase G>
auto findFirstSeparationPair(const G& graph)
    -> std::expected<std::optional<SeparationPair<typename G::NodeIdType>>,
        SeparationPairsError>
{
    return detail::findSeparationPair(graph);
}

template <GraphBase G>
std::expected<bool, SeparationPairsError> isTriconnected(const G& graph)
{
    auto result = findFirstSeparationPair(graph);

    if (!result.has_value()) {
        return std::unexpected(result.error());
    }

    return !result.value().has_value();
}

} // namespace MathUtils

#endif // SEPARATION_PAIRS_HPP
