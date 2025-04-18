/**
 *  ALGORITHM SHORT DESCRIPTION
 *  In DFS tree, a vertex u is an articulation point if one of the following two conditions is true.
 *
 *  - u is the root of the DFS tree and it has at least two children.
 *  - u is not the root of the DFS tree and it has a child v such that no vertex in the subtree rooted with
 *    v has a back edge to one of the ancestors in DFS tree of u.
 */

#include <cstdint>
#include <iostream>
#include <unordered_map>
#include "graph.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

using namespace MathUtils;

const auto graphLogger = spdlog::stdout_color_mt("GRAPH");

struct FindArticulationNodeMetaInfo {
    uint16_t m_children = 0; ///< The number of children Nodes in the DFS tree of the current node
    const Node* m_parent = nullptr; ///< The parent node of the current node. Initializes to nullptr
    int16_t m_discoveryTime = -1; ///< The discovery time of the current node in the DFS tree
    int16_t low = -1; ///< Lowest discovery time of the node reachable from this subtree
    bool visited = false; ///< Whether the current node was visited in the current DFS search
};

// This should at least be a protected method of graph
void findCutVertices(std::unordered_map<Node, FindArticulationNodeMetaInfo, Node>& nodesMetaInfoMap,
    const Node& parentNode, std::vector<Node>& articulationNodes)
{
    static uint16_t time = 0;

    auto& parentMetaInfo = nodesMetaInfoMap.at(parentNode);

    parentMetaInfo.visited = true;
    parentMetaInfo.m_discoveryTime = parentMetaInfo.low = time++;

    for (auto& child : parentNode.m_edges) {
        auto& currentChildNode = child.m_neighbour; // This may be shady since m_neighbour is a pointer!!!???
        auto& currentChildMetaInfo = nodesMetaInfoMap.at(currentChildNode);

        if (!currentChildMetaInfo.visited) {
            parentMetaInfo.m_children++;
            currentChildMetaInfo.m_parent = &parentNode;
            graphLogger->debug(
                std::format("Found node in DFS tree: {}, parent: {}", currentChildNode.tempName, parentNode.tempName));

            // Stepping deeper in the DFS tree
            findCutVertices(nodesMetaInfoMap, currentChildNode, articulationNodes);

            parentMetaInfo.low = std::min(parentMetaInfo.low, currentChildMetaInfo.low);

            // Check if the current node is an articulation node, with checking if it's not root and if
            // it has a back edge above its parent
            if (parentMetaInfo.m_parent && currentChildMetaInfo.low >= parentMetaInfo.m_discoveryTime) {
                graphLogger->debug("Found non-root articulation node");
                articulationNodes.push_back(parentNode);
            }
        }

        // Note that parrent node is allowed to be nullptr
        else if (const Node* parentPtr = parentMetaInfo.m_parent;
            parentPtr != nullptr && currentChildNode != *parentPtr) {
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
    std::unordered_map<Node, FindArticulationNodeMetaInfo, Node> nodesMetaInfoMap;
    std::vector<Node> articulationNodes; // TODO this could be optimized with reference wrapper

    for (auto& node : m_adjacencyList)
        nodesMetaInfoMap.insert({ node, {} });

    // Makes sure the algorithm works for disconnected graphs as well
    graphLogger->info("Articulation point search started based on Tarjan's recursive algorithm");
    for (auto& node : m_adjacencyList) {
        if (auto it = nodesMetaInfoMap.find(node); it != nodesMetaInfoMap.end()) {
            if (!it->second.visited) {
                findCutVertices(nodesMetaInfoMap, m_adjacencyList.front(), articulationNodes);
            }
        } else {
            std::cerr << node.tempName << " is not found in map while checking for disconnected graph" << std::endl;
        }
    }

    graphLogger->info("Articulation point search finished");
    return articulationNodes;
}
