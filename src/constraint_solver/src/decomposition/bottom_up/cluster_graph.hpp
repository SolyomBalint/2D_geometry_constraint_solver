#ifndef CLUSTER_GRAPH_HPP
#define CLUSTER_GRAPH_HPP

// General STD/STL headers
#include <expected>
#include <span>
#include <unordered_map>
#include <variant>
#include <vector>

// Custom headers
#include "cluster_types.hpp"
#include <structures/property_map.hpp>
#include <structures/simple_graph.hpp>

namespace Gcs {

enum class HNodeKind {
    Element,
    Cluster,
};

struct HNodeData {
    HNodeKind kind;
    std::variant<ConstraintGraph::NodeIdType, ClusterId> ref;
};

enum class ClusterGraphError {
    ClusterNotFound,
    ElementNotFound,
    EmptyCluster,
    DuplicateClusterInput,
    InvalidBipartiteState,
};

class ClusterGraph {
public:
    using Graph = MathUtils::SimpleGraph;

    static ClusterGraph fromConstraintGraph(const ConstraintGraph& graph);

    std::expected<ClusterId, ClusterGraphError> addCluster(
        std::span<const ConstraintGraph::NodeIdType> elements);

    std::expected<void, ClusterGraphError> removeCluster(ClusterId clusterId);

    [[nodiscard]] std::expected<std::vector<ConstraintGraph::NodeIdType>,
        ClusterGraphError>
    elementsOf(ClusterId clusterId) const;

    [[nodiscard]] std::vector<ClusterId> clustersContaining(
        ConstraintGraph::NodeIdType elementId) const;

    [[nodiscard]] std::vector<ClusterId> getAliveClusters() const;

    std::expected<ClusterId, ClusterGraphError> mergeThreeClusters(
        ClusterId c1, ClusterId c2, ClusterId c3);

    [[nodiscard]] const Graph& getGraph() const { return m_graph; }

private:
    [[nodiscard]] bool isValidBipartiteState() const;

    Graph m_graph;
    MathUtils::NodePropertyMap<HNodeData> m_hNodeData;

    std::unordered_map<ConstraintGraph::NodeIdType, Graph::NodeIdType>
        m_elementToHNode;
    std::unordered_map<ClusterId, Graph::NodeIdType> m_clusterToHNode;

    std::unordered_map<ClusterId, Cluster> m_clusters;

    int m_nextClusterId { 0 };
};

} // namespace Gcs

#endif // CLUSTER_GRAPH_HPP
