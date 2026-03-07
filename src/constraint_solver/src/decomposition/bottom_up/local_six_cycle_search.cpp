#include "local_six_cycle_search.hpp"

// General STD/STL headers
#include <algorithm>
#include <array>
#include <set>
#include <unordered_map>
#include <vector>

namespace Gcs {

namespace {

    struct LevelThreeBranch {
        ConstraintGraph::NodeIdType levelOneElement;
        ClusterId levelTwoCluster;
    };

    struct WitnessCanonicalKey {
        std::array<ClusterId, 3> clusters;
        std::array<ConstraintGraph::NodeIdType, 3> elements;

        bool operator<(const WitnessCanonicalKey& other) const
        {
            if (clusters < other.clusters) {
                return true;
            }
            if (other.clusters < clusters) {
                return false;
            }
            return elements < other.elements;
        }
    };

    // Create canonical ordering key for deterministic witness de-duplication.
    [[nodiscard]] WitnessCanonicalKey canonicalKey(
        const SixCycleWitness& witness)
    {
        std::array<ClusterId, 3> clusters { witness.ab, witness.bc,
            witness.ac };
        std::array<ConstraintGraph::NodeIdType, 3> elements {
            witness.a,
            witness.b,
            witness.c,
        };

        std::ranges::sort(clusters);
        std::ranges::sort(elements);
        return WitnessCanonicalKey {
            .clusters = clusters,
            .elements = elements,
        };
    }

} // namespace

std::vector<SixCycleWitness> findLocalSixCyclesAround(
    ClusterId clusterId, const ClusterGraph& clusterGraph)
{
    const auto rootElements = clusterGraph.elementsOf(clusterId);
    if (!rootElements.has_value()) {
        return {};
    }

    // BFS level accounting: level-3 elements mapped to all reaching branches.
    std::unordered_map<ConstraintGraph::NodeIdType,
        std::vector<LevelThreeBranch>>
        levelThreeHits;

    for (const auto& u : rootElements.value()) {
        const auto clustersAtLevelTwo = clusterGraph.clustersContaining(u);
        for (const auto& U : clustersAtLevelTwo) {
            if (U == clusterId) {
                continue;
            }

            const auto levelThreeElements = clusterGraph.elementsOf(U);
            if (!levelThreeElements.has_value()) {
                continue;
            }

            for (const auto& w : levelThreeElements.value()) {
                if (w == u) {
                    continue;
                }

                levelThreeHits[w].push_back(LevelThreeBranch {
                    .levelOneElement = u,
                    .levelTwoCluster = U,
                });
            }
        }
    }

    std::vector<SixCycleWitness> witnesses;
    std::set<WitnessCanonicalKey> seen;

    for (const auto& [w, branches] : levelThreeHits) {
        if (branches.size() < 2) {
            continue;
        }

        for (std::size_t i = 0; i < branches.size(); ++i) {
            for (std::size_t j = i + 1; j < branches.size(); ++j) {
                const auto& left = branches[i];
                const auto& right = branches[j];

                if (left.levelOneElement == right.levelOneElement) {
                    continue;
                }

                if (left.levelTwoCluster == right.levelTwoCluster) {
                    continue;
                }

                if (w == left.levelOneElement || w == right.levelOneElement) {
                    continue;
                }

                const SixCycleWitness witness {
                    .ab = clusterId,
                    .bc = left.levelTwoCluster,
                    .ac = right.levelTwoCluster,
                    .a = left.levelOneElement,
                    .b = w,
                    .c = right.levelOneElement,
                };

                const auto key = canonicalKey(witness);
                if (seen.contains(key)) {
                    continue;
                }

                seen.insert(key);
                witnesses.push_back(witness);
            }
        }
    }

    std::ranges::sort(
        witnesses, [](const SixCycleWitness& lhs, const SixCycleWitness& rhs) {
            return canonicalKey(lhs) < canonicalKey(rhs);
        });
    return witnesses;
}

} // namespace Gcs
