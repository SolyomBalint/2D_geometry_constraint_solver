#include "six_cycle_witness.hpp"

// General STD/STL headers
#include <algorithm>
#include <array>

namespace Gcs {

namespace {

    // Find a unique active size-2 cluster matching one element pair.
    [[nodiscard]] std::optional<ClusterId> findUniquePairCluster(
        ConstraintGraph::NodeIdType first, ConstraintGraph::NodeIdType second,
        const ClusterGraph& clusterGraph)
    {
        const std::array<ConstraintGraph::NodeIdType, 2> edgeElements {
            first,
            second,
        };
        const auto canonicalPair = makeCanonicalClusterElements(edgeElements);

        std::optional<ClusterId> match;
        for (const auto& candidate : clusterGraph.clustersContaining(first)) {
            const auto candidateElements = clusterGraph.elementsOf(candidate);
            if (!candidateElements.has_value()) {
                continue;
            }

            if (!areSameClusterElements(
                    canonicalPair, candidateElements.value())) {
                continue;
            }

            if (match.has_value()) {
                return std::nullopt;
            }

            match = candidate;
        }

        return match;
    }

} // namespace

std::optional<SixCycleWitness> findInitialSixCycleForTriangle(
    const MathUtils::Triangle<ConstraintGraph::NodeIdType>& triangle,
    const ClusterGraph& clusterGraph)
{
    const auto ab = findUniquePairCluster(triangle.a, triangle.b, clusterGraph);
    if (!ab.has_value()) {
        return std::nullopt;
    }

    const auto bc = findUniquePairCluster(triangle.b, triangle.c, clusterGraph);
    if (!bc.has_value()) {
        return std::nullopt;
    }

    const auto ac = findUniquePairCluster(triangle.a, triangle.c, clusterGraph);
    if (!ac.has_value()) {
        return std::nullopt;
    }

    std::array<ClusterId, 3> clusterIds { ab.value(), bc.value(), ac.value() };
    std::ranges::sort(clusterIds);
    const auto uniqueBegin = std::ranges::unique(clusterIds).begin();
    if (uniqueBegin != clusterIds.end()) {
        return std::nullopt;
    }

    return SixCycleWitness {
        .ab = ab.value(),
        .bc = bc.value(),
        .ac = ac.value(),
        .a = triangle.a,
        .b = triangle.b,
        .c = triangle.c,
    };
}

} // namespace Gcs
