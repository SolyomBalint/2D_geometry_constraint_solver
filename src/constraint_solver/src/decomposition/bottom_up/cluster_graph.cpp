#include "cluster_graph.hpp"

// General STD/STL headers
#include <algorithm>
#include <cassert>
#include <ranges>

namespace Gcs {

namespace {

    // Collect element IDs adjacent to a cluster node in canonical order.
    [[nodiscard]] std::vector<ConstraintGraph::NodeIdType>
    sortedUniqueElementsFromClusterNeighbors(const ClusterGraph::Graph& graph,
        const MathUtils::NodePropertyMap<HNodeData>& hNodeData,
        ClusterGraph::Graph::NodeIdType clusterHNode)
    {
        std::vector<ConstraintGraph::NodeIdType> neighborElements;

        for (const auto& incidentEdge : graph.getEdges(clusterHNode)) {
            const auto [source, target] = graph.getEndpoints(incidentEdge);
            const auto neighbor = (source == clusterHNode) ? target : source;

            const auto neighborData = hNodeData.get(neighbor).value().get();
            assert(neighborData.kind == HNodeKind::Element
                && "Cluster node must connect only to element nodes");

            neighborElements.push_back(
                std::get<ConstraintGraph::NodeIdType>(neighborData.ref));
        }

        std::ranges::sort(neighborElements);
        const auto uniqueBegin = std::ranges::unique(neighborElements).begin();
        neighborElements.erase(uniqueBegin, neighborElements.end());

        return neighborElements;
    }

} // namespace

ClusterGraph ClusterGraph::fromConstraintGraph(const ConstraintGraph& graph)
{
    ClusterGraph clusterGraph;

    for (const auto& elementId : graph.getGraph().getNodes()) {
        const auto hNode = clusterGraph.m_graph.addNode();

        clusterGraph.m_hNodeData.set(hNode,
            HNodeData {
                .kind = HNodeKind::Element,
                .ref = elementId,
            });
        clusterGraph.m_elementToHNode.emplace(elementId, hNode);
    }

    assert(clusterGraph.isValidBipartiteState()
        && "ClusterGraph initialization must preserve bipartite invariants");

    return clusterGraph;
}

std::expected<ClusterId, ClusterGraphError> ClusterGraph::addCluster(
    std::span<const ConstraintGraph::NodeIdType> elements)
{
    auto clusterElements = makeCanonicalClusterElements(elements);
    if (clusterElements.empty()) {
        return std::unexpected(ClusterGraphError::EmptyCluster);
    }

    for (const auto& elementId : clusterElements) {
        if (!m_elementToHNode.contains(elementId)) {
            return std::unexpected(ClusterGraphError::ElementNotFound);
        }
    }

    const ClusterId clusterId { .value = m_nextClusterId++ };
    const auto clusterHNode = m_graph.addNode();

    m_hNodeData.set(clusterHNode,
        HNodeData {
            .kind = HNodeKind::Cluster,
            .ref = clusterId,
        });
    m_clusterToHNode.emplace(clusterId, clusterHNode);
    m_clusters.emplace(clusterId,
        Cluster {
            .id = clusterId,
            .elements = clusterElements,
        });

    for (const auto& elementId : clusterElements) {
        const auto elementHNode = m_elementToHNode.at(elementId);
        if (!m_graph.addEdge(clusterHNode, elementHNode).has_value()) {
            return std::unexpected(ClusterGraphError::InvalidBipartiteState);
        }
    }

    assert(isValidBipartiteState()
        && "addCluster must preserve bipartite invariants");

    return clusterId;
}

std::expected<void, ClusterGraphError> ClusterGraph::removeCluster(
    ClusterId clusterId)
{
    if (!m_clusterToHNode.contains(clusterId)
        || !m_clusters.contains(clusterId)) {
        return std::unexpected(ClusterGraphError::ClusterNotFound);
    }

    const auto clusterHNode = m_clusterToHNode.at(clusterId);
    if (!m_graph.removeNode(clusterHNode).has_value()) {
        return std::unexpected(ClusterGraphError::InvalidBipartiteState);
    }

    m_clusterToHNode.erase(clusterId);
    m_clusters.erase(clusterId);

    if (!m_hNodeData.erase(clusterHNode).has_value()) {
        return std::unexpected(ClusterGraphError::InvalidBipartiteState);
    }

    assert(isValidBipartiteState()
        && "removeCluster must preserve bipartite invariants");

    return {};
}

std::expected<std::vector<ConstraintGraph::NodeIdType>, ClusterGraphError>
ClusterGraph::elementsOf(ClusterId clusterId) const
{
    if (!m_clusters.contains(clusterId)) {
        return std::unexpected(ClusterGraphError::ClusterNotFound);
    }

    return m_clusters.at(clusterId).elements;
}

std::vector<ClusterId> ClusterGraph::clustersContaining(
    ConstraintGraph::NodeIdType elementId) const
{
    if (!m_elementToHNode.contains(elementId)) {
        return {};
    }

    const auto elementHNode = m_elementToHNode.at(elementId);
    std::vector<ClusterId> clusterIds;

    for (const auto& incidentEdge : m_graph.getEdges(elementHNode)) {
        const auto [source, target] = m_graph.getEndpoints(incidentEdge);
        const auto neighbor = (source == elementHNode) ? target : source;

        const auto neighborData = m_hNodeData.get(neighbor).value().get();
        if (neighborData.kind != HNodeKind::Cluster) {
            continue;
        }

        clusterIds.push_back(std::get<ClusterId>(neighborData.ref));
    }

    std::ranges::sort(clusterIds);
    return clusterIds;
}

std::vector<ClusterId> ClusterGraph::getAliveClusters() const
{
    std::vector<ClusterId> clusterIds;
    clusterIds.reserve(m_clusters.size());
    for (const auto& [clusterId, cluster] : m_clusters) {
        (void)cluster;
        clusterIds.push_back(clusterId);
    }

    std::ranges::sort(clusterIds);
    return clusterIds;
}

std::expected<ClusterId, ClusterGraphError> ClusterGraph::mergeThreeClusters(
    ClusterId c1, ClusterId c2, ClusterId c3)
{
    if (c1 == c2 || c1 == c3 || c2 == c3) {
        return std::unexpected(ClusterGraphError::DuplicateClusterInput);
    }

    if (!m_clusters.contains(c1) || !m_clusters.contains(c2)
        || !m_clusters.contains(c3)) {
        return std::unexpected(ClusterGraphError::ClusterNotFound);
    }

    const auto mergedElements = clusterElementsUnion(m_clusters.at(c1).elements,
        m_clusters.at(c2).elements, m_clusters.at(c3).elements);

    const auto mergedClusterId = addCluster(mergedElements);
    if (!mergedClusterId.has_value()) {
        return std::unexpected(mergedClusterId.error());
    }

    if (!removeCluster(c1).has_value() || !removeCluster(c2).has_value()
        || !removeCluster(c3).has_value()) {
        return std::unexpected(ClusterGraphError::InvalidBipartiteState);
    }

    assert(isValidBipartiteState()
        && "mergeThreeClusters must preserve bipartite invariants");

    return mergedClusterId.value();
}

bool ClusterGraph::isValidBipartiteState() const
{
    for (const auto& [elementId, elementHNode] : m_elementToHNode) {
        (void)elementId;
        if (!m_graph.hasNode(elementHNode)) {
            return false;
        }

        const auto data = m_hNodeData.get(elementHNode);
        if (!data.has_value()) {
            return false;
        }

        if (data.value().get().kind != HNodeKind::Element) {
            return false;
        }
    }

    for (const auto& [clusterId, clusterHNode] : m_clusterToHNode) {
        if (!m_graph.hasNode(clusterHNode)) {
            return false;
        }

        const auto data = m_hNodeData.get(clusterHNode);
        if (!data.has_value()) {
            return false;
        }

        if (data.value().get().kind != HNodeKind::Cluster) {
            return false;
        }

        if (std::get<ClusterId>(data.value().get().ref) != clusterId) {
            return false;
        }
    }

    if (m_clusterToHNode.size() != m_clusters.size()) {
        return false;
    }

    for (const auto& [clusterId, cluster] : m_clusters) {
        if (!m_clusterToHNode.contains(clusterId)) {
            return false;
        }

        if (cluster.elements.empty()) {
            return false;
        }

        const auto clusterHNode = m_clusterToHNode.at(clusterId);
        const auto elementNeighbors = sortedUniqueElementsFromClusterNeighbors(
            m_graph, m_hNodeData, clusterHNode);

        if (!areSameClusterElements(cluster.elements, elementNeighbors)) {
            return false;
        }
    }

    for (const auto& edgeId : m_graph.getEdges()) {
        const auto [source, target] = m_graph.getEndpoints(edgeId);
        const auto sourceData = m_hNodeData.get(source);
        const auto targetData = m_hNodeData.get(target);

        if (!sourceData.has_value() || !targetData.has_value()) {
            return false;
        }

        if (sourceData.value().get().kind == targetData.value().get().kind) {
            return false;
        }
    }

    return true;
}

} // namespace Gcs
