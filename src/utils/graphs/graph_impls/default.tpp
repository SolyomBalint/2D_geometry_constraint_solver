namespace MathUtils {

// Template class DefaultUndirectedGraph implementation

template <typename NodeStoredObject, typename EdgeStoredObject>
typename DefaultUndirectedGraph<NodeStoredObject, EdgeStoredObject>::NodeType&
DefaultUndirectedGraph<NodeStoredObject, EdgeStoredObject>::addNode(std::shared_ptr<NodeStoredObject> obj)
{
    auto nodeId = Common::generateUuidMt19937();
    auto [it, ret] = nodes.emplace(nodeId, NodeType { { nodeId }, obj });

    adjacencyLists.emplace(nodeId, std::unordered_set<DefaultEdge::EdgeId> {});

    return it->second;
}

template <typename NodeStoredObject, typename EdgeStoredObject>
typename DefaultUndirectedGraph<NodeStoredObject, EdgeStoredObject>::EdgeType&
DefaultUndirectedGraph<NodeStoredObject, EdgeStoredObject>::addEdge(
    const NodeType& node1, const NodeType& node2, std::shared_ptr<EdgeStoredObject> edgeObj)
{
    const auto node1Id = node1.getId();
    const auto node2Id = node2.getId();

    {
        auto node1It = nodes.find(node1Id);
        auto node2It = nodes.find(node2Id);

        if (node1It == nodes.end() or node2It == nodes.end()) {
            throw std::invalid_argument("Input nodes do not exist in the graph, it is impossible to create the edge");
        }
    }

    auto edgeId = Common::generateUuidMt19937();

    auto [it, ret] = edges.emplace(edgeId, EdgeType { edgeId, edgeObj });
    neighbours.emplace(edgeId, std::make_pair(node1Id, node2Id));

    adjacencyLists[node1Id].insert(edgeId);
    adjacencyLists[node2Id].insert(edgeId);

    return it->second;
}

template <typename NodeStoredObject, typename EdgeStoredObject>
std::vector<typename DefaultUndirectedGraph<NodeStoredObject, EdgeStoredObject>::NodeType>
DefaultUndirectedGraph<NodeStoredObject, EdgeStoredObject>::getCutVertices() const
{
    // Skeleton implementation
    throw std::logic_error("Method not implemented");
}

template <NodeStoredObj, EdgeStoredObj>
using Graph = GraphInterface<DefaultUndirectedGraph<NodeStoredObj, EdgeStoredObj>, DefaultNode, NodeStoredObj,
    DefaultEdge, EdgeStoredObj>;

} // namespace MathUtils
