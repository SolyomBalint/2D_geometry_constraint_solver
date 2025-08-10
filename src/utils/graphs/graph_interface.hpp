#ifndef GRAPH_INTERFACE_HPP
#define GRAPH_INTERFACE_HPP

#include <concepts>
#include <memory>
#include <ranges>
#include <utility>
#include <vector>
#include <algorithm>

namespace MathUtils {

// TODO conecpts for Node, and Edge
template <typename NodeImpl, typename StoredObjectType = void> class NodeInterface {
private:
    NodeImpl impl_;
    std::shared_ptr<StoredObjectType> storedObject;

public:
    NodeInterface() = default;
    NodeInterface(NodeImpl impl, std::shared_ptr<StoredObjectType> obj)
        : impl_(impl)
        , storedObject(obj)
    {
    }
    NodeInterface(const NodeInterface&) = default;
    NodeInterface& operator=(const NodeInterface&) = default;
    NodeInterface(NodeInterface&&) = default;
    NodeInterface& operator=(NodeInterface&&) = default;

    auto getId() const { return impl_.getId(); }

    std::shared_ptr<StoredObjectType> getStoredObj() { return storedObject; }
};

template <typename EdgeImpl, typename StoredObjectType = void> class EdgeInterface {
private:
    EdgeImpl impl_;
    std::shared_ptr<StoredObjectType> storedObject;

public:
    EdgeInterface() = default;

    EdgeInterface(EdgeImpl impl, std::shared_ptr<StoredObjectType> obj)
        : impl_(impl)
        , storedObject(obj)
    {
    }

    EdgeInterface(const EdgeInterface&) = default;
    EdgeInterface& operator=(const EdgeInterface&) = default;
    EdgeInterface(EdgeInterface&&) = default;
    EdgeInterface& operator=(EdgeInterface&&) = default;

    auto getId() const { return impl_.getId(); }

    std::shared_ptr<StoredObjectType> getStoredObj() { return storedObject; }
};

/**
 * @concept GraphImplRequirements
 * @brief Defines the interface requirements for graph data structure implementations
 *
 * This concept specifies the minimum interface that a graph implementation must provide
 * to be compatible with graph algorithms. It ensures type safety by requiring specific
 * node and edge types, and mandates essential graph operations.
 *
 * @tparam GraphType The graph implementation type that must satisfy these requirements
 * @tparam NodeType The type used to identify or label nodes in the graph
 * @tparam NodeStoredObject The data type stored within each node
 * @tparam EdgeType The type used to identify or label edges in the graph
 * @tparam EdgeStoredObject The data type stored within each edge
 *
 * ## Type Requirements
 *
 * The GraphType must define the following type aliases:
 * - `node_type`: Must be exactly `Node<NodeType, NodeStoredObject>`
 * - `edge_type`: Must be exactly `Edge<EdgeType, EdgeStoredObject>`
 *
 * ## Required Operations
 *
 * ### Node Operations
 * - **addNode(NodeStoredObject)**: Adds a new node to the graph
 *   - **Parameters**: Node data to store
 *   - **Returns**: Reference to the newly created node
 *
 * ### Edge Operations
 * - **addEdge(const node_type&, const node_type&, EdgeStoredObject)**: Connects two nodes
 *   - **Parameters**: Source node, target node, edge data to store
 *   - **Returns**: Const reference to the newly created edge
 *
 * ### Graph Analysis Operations
 * - **getCutVertices()**: Finds articulation points in the graph
 *   - **Returns**: Range of nodes that are cut vertices
 *   - **Range Element Type**: Must be `GraphType::node_type`
 *
 * ## Usage Example
 *
 * @code{.cpp}
 * template<GraphImplRequirements<int, std::string, char, double> GraphImpl>
 * void processGraph(GraphImpl& graph) {
 *     auto& node1 = graph.addNode(std::string("Node1"));
 *     auto& node2 = graph.addNode(std::string("Node2"));
 *     auto& edge = graph.addEdge(node1, node2, 1.5);
 *
 *     auto cutVertices = graph.getCutVertices();
 *     for (const auto& vertex : cutVertices) {
 *         // Process cut vertices
 *     }
 * }
 * @endcode
 *
 * @see Node
 * @see Edge
 * @ingroup graph_concepts
 *
 * @since C++20
 * @headerfile "graph_concepts.h"
 */
template <typename GraphType, typename NodeType, typename NodeStoredObject, typename EdgeType,
    typename EdgeStoredObject>
concept GraphImplRequirements = requires(
    GraphType graph_t, NodeStoredObject node_stored_t, EdgeStoredObject edge_stored_t) {
    requires std::same_as<typename GraphType::NodeType, NodeInterface<NodeType, NodeStoredObject>>;
    requires std::same_as<typename GraphType::EdgeType, EdgeInterface<EdgeType, EdgeStoredObject>>;

    {
        graph_t.addNode(std::declval<std::shared_ptr<NodeStoredObject>>())
    } -> std::same_as<NodeInterface<NodeType, NodeStoredObject>&>;

    {
        graph_t.addEdge(std::declval<const typename GraphType::NodeType&>(),
            std::declval<const typename GraphType::NodeType&>(), std::declval<std::shared_ptr<EdgeStoredObject>>())
    } -> std::same_as<EdgeInterface<EdgeType, EdgeStoredObject>&>;

    { graph_t.getCutVertices() } -> std::ranges::range;

    requires std::same_as<std::ranges::range_value_t<decltype(graph_t.getCutVertices())>, typename GraphType::NodeType>;
};

template <typename GraphImpl, typename NodeImpl, typename NodeStoredObject, typename EdgeImpl,
    typename EdgeStoredObject>
    requires GraphImplRequirements<GraphImpl, NodeImpl, NodeStoredObject, EdgeImpl, EdgeStoredObject>
class GraphInterface {
private:
    GraphImpl impl_;

public:
    GraphInterface() = default;

    GraphInterface(const GraphInterface&) = delete;
    GraphInterface& operator=(const GraphInterface&) = delete;
    GraphInterface(GraphInterface&&) = delete;
    GraphInterface& operator=(GraphInterface&&) = delete;

    using NodeType = NodeInterface<NodeImpl, NodeStoredObject>;
    using EdgeType = EdgeInterface<EdgeImpl, EdgeStoredObject>;

    NodeType& addNode(std::shared_ptr<NodeStoredObject> obj) { return impl_.addNode(obj); }

    EdgeType& addEdge(const NodeType& node1, const NodeType& node2, std::shared_ptr<EdgeStoredObject> obj)
    {
        return impl_.addEdge(node1, node2, obj);
    }

    auto getCutVertices() const { return impl_.getCutVertices(); }
};

}
#endif // GRAPH_INTERFACE_HPP
