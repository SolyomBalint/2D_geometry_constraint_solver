/**
 *  ALGORITHM SHORT DESCRIPTION
 *  In DFS tree, a vertex u is an articulation point if one of the following two conditions is true.
 *
 *  - u is the root of the DFS tree and it has at least two children.
 *  - u is not the root of the DFS tree and it has a child v such that no vertex in the subtree rooted with
 *    v has a back edge to one of the ancestors in DFS tree of u.
 */

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utils/graph.hpp>

using namespace MathUtils;

struct FindArticulationNodeMetaInfo {
    uint16_t m_children = 0; ///< The number of children Nodes in the DFS tree of the current node
    std::shared_ptr<Node> m_parent; ///< The parent node of the current node. Initializes to nullptr
    uint16_t m_discoveryTime = -1; ///< The discovery time of the current node in the DFS tree
    uint16_t low = -1; ///< Lowest discovery time of the node reachable from this subtree
    bool visited = false; ///< Whether the current node was visited in the current DFS search
};

// This should at least be a protected method of graph
void findCutVertices(std::unordered_map<Node, FindArticulationNodeMetaInfo>& nodesMetaInfoMap,
    const Node& parentNode, std::vector<Node>& articulationNodes)
{
    static uint16_t time = 0;

    auto& parentMetaInfo = nodesMetaInfoMap.at(parentNode);

    parentMetaInfo.visited = true;
    parentMetaInfo.m_discoveryTime = parentMetaInfo.low = time++;

    for (auto& child : parentNode.m_edges) {
        auto& currentChildNode = child.m_neighbour; // This may be shady since m_neighbour is a pointer!!!???
        if (!currentChildNode) {
            graphLogger->error("Trying to use nullptr neighbour node pointer during articulation node search, exiting");
            std::exit(-1);
        }
        auto& currentChildMetaInfo = nodesMetaInfoMap.at(*currentChildNode);

        if (!currentChildMetaInfo.visited) {
            parentMetaInfo.m_children++;
            currentChildMetaInfo.m_parent = std::make_shared<Node>(parentNode);
            graphLogger->debug("Found node in DFS tree");

            // Stepping deeper in the DFS tree
            findCutVertices(nodesMetaInfoMap, *currentChildNode, articulationNodes);

            parentMetaInfo.low = std::min(parentMetaInfo.low, currentChildMetaInfo.low);

            // Check if the current node is an articulation node, with checking if it's not root and if
            // it has a back edge above its parent
            if (parentMetaInfo.m_parent && currentChildMetaInfo.low >= parentMetaInfo.m_discoveryTime) {
                graphLogger->debug("Found non-root articulation node");
                articulationNodes.push_back(parentNode);
            }
        }

        // Note that parrent node is allowed to be nullptr
        else if (*currentChildNode != *(parentMetaInfo.m_parent)) {
            parentMetaInfo.low = std::min(parentMetaInfo.low, currentChildMetaInfo.m_discoveryTime);
        }
    }

    // Check if the current node is an articulation node, with checking if it's  root and has two ore more children
    if (!parentMetaInfo.m_parent && parentMetaInfo.m_children > 1) {
        graphLogger->debug("Found root articulation node");
        articulationNodes.push_back(parentNode);
    }
};

std::vector<Node> UndirectedGraph::getCutVertices()
{
    std::unordered_map<Node, FindArticulationNodeMetaInfo> nodesMetaInfoMap;
    std::vector<Node> articulationNodes; // TODO this could be optimized with reference wrapper

    for (auto& node : m_adjacencyList)
        nodesMetaInfoMap.insert({ node, {} });

    // Makes sure the algorithm works for disconnected graphs as well
    graphLogger->info("Articulation point search started based on Tarjan's recursive algorithm");
    for (auto& node : m_adjacencyList) {
        if (!nodesMetaInfoMap.find(node)->second.visited) {
            findCutVertices(nodesMetaInfoMap, node, articulationNodes);
        }
    }

    graphLogger->info("Articulation point search finished");
    return articulationNodes;
}
