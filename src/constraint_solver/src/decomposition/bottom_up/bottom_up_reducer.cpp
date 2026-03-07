#include "bottom_up_reducer.hpp"

// General STD/STL headers
#include <algorithm>
#include <array>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Custom headers
#include "initial_edge_clusters.hpp"
#include "local_six_cycle_search.hpp"
#include "producer_update.hpp"
#include "six_cycle_witness.hpp"
#include <structures/graph_algorithms.hpp>

namespace Gcs {

namespace {

    // Check if canonical element set lhs is a strict subset of rhs.
    [[nodiscard]] bool isStrictSubset(
        std::span<const ConstraintGraph::NodeIdType> lhs,
        std::span<const ConstraintGraph::NodeIdType> rhs)
    {
        if (lhs.size() >= rhs.size()) {
            return false;
        }

        return std::ranges::includes(rhs, lhs);
    }

    // Check whether all witness clusters are still alive.
    [[nodiscard]] bool areWitnessClustersAlive(
        const SixCycleWitness& witness, const ClusterGraph& clusterGraph)
    {
        return clusterGraph.elementsOf(witness.ab).has_value()
            && clusterGraph.elementsOf(witness.bc).has_value()
            && clusterGraph.elementsOf(witness.ac).has_value();
    }

    // Check whether one cluster currently contains a given element.
    [[nodiscard]] bool clusterContainsElement(ClusterId clusterId,
        ConstraintGraph::NodeIdType elementId, const ClusterGraph& clusterGraph)
    {
        const auto elements = clusterGraph.elementsOf(clusterId);
        if (!elements.has_value()) {
            return false;
        }

        return std::ranges::binary_search(elements.value(), elementId);
    }

    // Check whether a merge can be represented in the current producer map.
    [[nodiscard]] bool isProducerCompatibleMerge(
        const std::array<ClusterId, 3>& inputs,
        const std::array<std::vector<ConstraintGraph::NodeIdType>, 3>&
            inputElements,
        const ClusterGraph& clusterGraph, const ProducerMap& producer)
    {
        const bool hasFirst = producer.contains(inputs[0]);
        const bool hasSecond = producer.contains(inputs[1]);
        const bool hasThird = producer.contains(inputs[2]);

        const bool anyProduced = hasFirst || hasSecond || hasThird;
        const bool allProduced = hasFirst && hasSecond && hasThird;

        if (anyProduced && !allProduced) {
            for (std::size_t index = 0; index < inputs.size(); ++index) {
                if (producer.contains(inputs[index])) {
                    continue;
                }

                if (inputElements[index].size() != 2
                    && inputElements[index].size() != 3) {
                    return false;
                }
            }
            return true;
        }

        if (allProduced) {
            return true;
        }

        const auto firstElements = clusterGraph.elementsOf(inputs[0]);
        const auto secondElements = clusterGraph.elementsOf(inputs[1]);
        const auto thirdElements = clusterGraph.elementsOf(inputs[2]);
        if (!firstElements.has_value() || !secondElements.has_value()
            || !thirdElements.has_value()) {
            return false;
        }

        const auto firstUnion = clusterElementsUnion(
            firstElements.value(), secondElements.value());
        const auto outputElements
            = clusterElementsUnion(firstUnion, thirdElements.value());
        return outputElements.size() == 3;
    }

    // Collect canonical element sets for each input cluster before merging.
    [[nodiscard]] std::optional<
        std::array<std::vector<ConstraintGraph::NodeIdType>, 3>>
    collectMergeInputElements(const std::array<ClusterId, 3>& inputs,
        const ClusterGraph& clusterGraph)
    {
        const auto firstElements = clusterGraph.elementsOf(inputs[0]);
        const auto secondElements = clusterGraph.elementsOf(inputs[1]);
        const auto thirdElements = clusterGraph.elementsOf(inputs[2]);
        if (!firstElements.has_value() || !secondElements.has_value()
            || !thirdElements.has_value()) {
            return std::nullopt;
        }

        return std::array<std::vector<ConstraintGraph::NodeIdType>, 3> {
            firstElements.value(),
            secondElements.value(),
            thirdElements.value(),
        };
    }

    // Revalidate one local six-cycle witness against the current H state.
    [[nodiscard]] bool isValidLocalWitness(
        const SixCycleWitness& witness, const ClusterGraph& clusterGraph)
    {
        if (witness.ab == witness.bc || witness.ab == witness.ac
            || witness.bc == witness.ac) {
            return false;
        }

        if (witness.a == witness.b || witness.a == witness.c
            || witness.b == witness.c) {
            return false;
        }

        if (!areWitnessClustersAlive(witness, clusterGraph)) {
            return false;
        }

        return clusterContainsElement(witness.ab, witness.a, clusterGraph)
            && clusterContainsElement(witness.ab, witness.c, clusterGraph)
            && clusterContainsElement(witness.bc, witness.a, clusterGraph)
            && clusterContainsElement(witness.bc, witness.b, clusterGraph)
            && clusterContainsElement(witness.ac, witness.b, clusterGraph)
            && clusterContainsElement(witness.ac, witness.c, clusterGraph);
    }

    // Apply local BFS-based six-cycle rewrites until no queued candidates
    // remain.
    void applyLocalRewriteLoop(ClusterGraph& clusterGraph,
        ProducerMap& producer, std::queue<ClusterId>& workQueue,
        std::unordered_set<ClusterId>& queued)
    {
        while (!workQueue.empty()) {
            const auto candidate = workQueue.front();
            workQueue.pop();
            queued.erase(candidate);

            if (!clusterGraph.elementsOf(candidate).has_value()) {
                continue;
            }

            const auto witnesses
                = findLocalSixCyclesAround(candidate, clusterGraph);
            for (const auto& witness : witnesses) {
                if (!isValidLocalWitness(witness, clusterGraph)) {
                    continue;
                }

                const std::array<ClusterId, 3> inputs {
                    witness.ab,
                    witness.bc,
                    witness.ac,
                };

                const auto inputElements
                    = collectMergeInputElements(inputs, clusterGraph);
                if (!inputElements.has_value()) {
                    continue;
                }

                if (!isProducerCompatibleMerge(inputs, inputElements.value(),
                        clusterGraph, producer)) {
                    continue;
                }

                const auto mergedCluster = clusterGraph.mergeThreeClusters(
                    inputs[0], inputs[1], inputs[2]);
                if (!mergedCluster.has_value()) {
                    continue;
                }

                const auto outputElements
                    = clusterGraph.elementsOf(mergedCluster.value());
                if (!outputElements.has_value()) {
                    throw std::runtime_error(
                        "Merged cluster missing from cluster graph");
                }

                if (!updateProducerAfterMergeThree(mergedCluster.value(),
                        inputs, inputElements.value(), outputElements.value(),
                        producer)
                        .has_value()) {
                    throw std::runtime_error(
                        "Failed to update producer after local merge");
                }

                if (!queued.contains(mergedCluster.value())) {
                    workQueue.push(mergedCluster.value());
                    queued.insert(mergedCluster.value());
                }
            }
        }
    }

    // Select alive clusters that are maximal by element-set inclusion.
    [[nodiscard]] std::vector<ClusterId> maximalAliveClusters(
        const std::vector<ClusterId>& aliveClusters,
        const ClusterGraph& clusterGraph)
    {
        std::unordered_map<ClusterId, std::vector<ConstraintGraph::NodeIdType>>
            clusterElements;
        clusterElements.reserve(aliveClusters.size());

        for (const auto& clusterId : aliveClusters) {
            const auto elements = clusterGraph.elementsOf(clusterId);
            if (!elements.has_value()) {
                continue;
            }
            clusterElements.emplace(clusterId, elements.value());
        }

        std::vector<ClusterId> maximal;
        maximal.reserve(clusterElements.size());

        for (const auto& clusterId : aliveClusters) {
            if (!clusterElements.contains(clusterId)) {
                continue;
            }

            bool isSubsetOfAnother = false;
            for (const auto& otherId : aliveClusters) {
                if (otherId == clusterId
                    || !clusterElements.contains(otherId)) {
                    continue;
                }

                if (isStrictSubset(clusterElements.at(clusterId),
                        clusterElements.at(otherId))) {
                    isSubsetOfAnother = true;
                    break;
                }
            }

            if (!isSubsetOfAnother) {
                maximal.push_back(clusterId);
            }
        }

        return maximal;
    }

} // namespace

BottomUpReductionResult reduceBottomUp(const ConstraintGraph& graph)
{
    ClusterGraph clusterGraph = ClusterGraph::fromConstraintGraph(graph);
    ProducerMap producer;
    std::queue<ClusterId> workQueue;
    std::unordered_set<ClusterId> queued;

    if (!addInitialEdgeClusters(graph, clusterGraph).has_value()) {
        throw std::runtime_error("Failed to create initial edge clusters");
    }

    const auto triangles = MathUtils::findTriangles(graph.getGraph());
    for (const auto& triangle : triangles) {
        const auto witness
            = findInitialSixCycleForTriangle(triangle, clusterGraph);
        if (!witness.has_value()) {
            continue;
        }

        const std::array<ClusterId, 3> inputs {
            witness->ab,
            witness->bc,
            witness->ac,
        };

        const auto inputElements
            = collectMergeInputElements(inputs, clusterGraph);
        if (!inputElements.has_value()) {
            continue;
        }

        if (!isProducerCompatibleMerge(
                inputs, inputElements.value(), clusterGraph, producer)) {
            continue;
        }

        const auto mergedCluster
            = clusterGraph.mergeThreeClusters(inputs[0], inputs[1], inputs[2]);
        if (!mergedCluster.has_value()) {
            continue;
        }

        const auto outputElements
            = clusterGraph.elementsOf(mergedCluster.value());
        if (!outputElements.has_value()) {
            throw std::runtime_error(
                "Merged cluster missing from cluster graph");
        }

        if (!updateProducerAfterMergeThree(mergedCluster.value(), inputs,
                inputElements.value(), outputElements.value(), producer)
                .has_value()) {
            throw std::runtime_error("Failed to update producer after merge");
        }

        if (!queued.contains(mergedCluster.value())) {
            workQueue.push(mergedCluster.value());
            queued.insert(mergedCluster.value());
        }
    }

    applyLocalRewriteLoop(clusterGraph, producer, workQueue, queued);

    BottomUpReductionResult result {
        .remainingClusters = clusterGraph.getAliveClusters(),
        .rootPlans = {},
    };

    const auto maximalClusters
        = maximalAliveClusters(result.remainingClusters, clusterGraph);
    result.rootPlans.reserve(maximalClusters.size());
    for (const auto& clusterId : maximalClusters) {
        if (!producer.contains(clusterId)) {
            continue;
        }

        result.rootPlans.push_back(producer.at(clusterId));
    }

    return result;
}

} // namespace Gcs
