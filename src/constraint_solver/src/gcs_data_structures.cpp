#include "gcs_data_structures.hpp"
#include "constraints.hpp"
#include "elements.hpp"

// General STD/STL headers
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// Custom headers
#include <structures/graph.hpp>
#include <structures/graph_algorithms.hpp>
#include <structures/separation_pairs.hpp>

namespace Gcs {

ConstraintGraph::EdgeIdType ConstraintGraph::addVirtualEdge(
    NodeIdType s, NodeIdType t)
{
    const auto& edge = m_constraintGraph.addEdge(s, t);

    if (edge.has_value()) {
        m_virtualEdges.insert(edge.value());
    } else {
        throw std::runtime_error("Failed to insert virtual edge");
    }

    return edge.value();
}

ConstraintGraphError ConstraintGraph::removeVirtualEdge(EdgeIdType virtualEdge)
{
    m_virtualEdges.erase(virtualEdge);
    const auto& err = m_constraintGraph.removeEdge(virtualEdge);
    if (!err.has_value()) {
        throw std::runtime_error("Virtual Edge not found");
    }
    return ConstraintGraphError::OK;
}
ConstraintGraphError ConstraintGraph::removeElement(NodeIdType node)
{
    if (!m_constraintGraph.hasNode(node))
        return ConstraintGraphError::NodeNotFound;

    // Remove constraint property entries for all incident edges
    for (const auto& edge : m_constraintGraph.getEdges(node)) {
        m_constraintEdgeMap.erase(edge);
        m_virtualEdges.erase(edge);
    }

    // Remove the element property entry
    m_elementNodeMap.erase(node);

    // Remove the node (and its incident edges) from the graph
    m_constraintGraph.removeNode(node);

    return ConstraintGraphError::OK;
}

ConstraintGraphError ConstraintGraph::removeConstraintEdge(EdgeIdType edge)
{
    if (!m_constraintGraph.hasEdge(edge))
        return ConstraintGraphError::EdgeNotFound;

    m_constraintEdgeMap.erase(edge);
    m_virtualEdges.erase(edge);
    m_constraintGraph.removeEdge(edge);

    return ConstraintGraphError::OK;
}

std::shared_ptr<Element> ConstraintGraph::getElement(NodeIdType node) const
{
    auto result = m_elementNodeMap.get(node);
    if (result.has_value())
        return result.value();
    return nullptr;
}

std::shared_ptr<Constraint> ConstraintGraph::getConstraintForEdge(
    EdgeIdType edge) const
{
    auto result = m_constraintEdgeMap.get(edge);
    if (result.has_value())
        return result.value();
    return nullptr;
}

ConstraintGraphError ConstraintGraph::addElement(
    Graph::NodeIdType node, std::shared_ptr<Element> element)
{
    if (!m_constraintGraph.hasNode(node))
        return ConstraintGraphError::NodeNotFound;

    m_elementNodeMap.set(node, std::move(element));
    return ConstraintGraphError::OK;
}

ConstraintGraphError ConstraintGraph::addConstraint(
    Graph::EdgeIdType edge, std::shared_ptr<Constraint> constraint)
{
    if (!m_constraintGraph.hasEdge(edge))
        return ConstraintGraphError::EdgeNotFound;

    if (m_virtualEdges.contains(edge)) {
        throw std::runtime_error("Virtual edges cannot carry constraints.");
    }

    m_constraintEdgeMap.set(edge, std::move(constraint));
    return ConstraintGraphError::OK;
}

SeparationGraphInfo
ConstraintGraph::mapSubGraphElementsAndConstraintsToOriginal(
    const MathUtils::SubGraph<Graph>& subGraph, NodeIdType originalSepNodeA,
    NodeIdType originalSepNodeB)
{
    ConstraintGraph result;

    // Map subgraph local node IDs to result graph node IDs so that edges
    // can be reconnected correctly in the new graph.
    std::unordered_map<NodeIdType, NodeIdType> subLocalToResultNode;

    for (const auto& localNode : subGraph.getNodes()) {
        auto resultNode = result.m_constraintGraph.addNode();
        subLocalToResultNode[localNode] = resultNode;

        auto originalNode = subGraph.originalNodeId(localNode);

        if (!originalNode.has_value()) {
            throw std::runtime_error(
                "mapSubGraphElementsAndConstraintsToOriginal: "
                "subgraph node has no mapping to original graph");
        }

        auto element = m_elementNodeMap.get(originalNode.value());

        if (element.has_value()) {
            result.m_elementNodeMap.set(resultNode, element.value().get());
        }
    }

    for (const auto& localEdge : subGraph.getEdges()) {
        auto [localS, localT] = subGraph.getEndpoints(localEdge);

        auto resultS = subLocalToResultNode.at(localS);
        auto resultT = subLocalToResultNode.at(localT);

        auto resultEdge = result.m_constraintGraph.addEdge(resultS, resultT);

        if (!resultEdge.has_value()) {
            throw std::runtime_error(
                "mapSubGraphElementsAndConstraintsToOriginal: "
                "failed to add edge to result graph");
        }

        auto originalEdge = subGraph.originalEdgeId(localEdge);

        if (!originalEdge.has_value()) {
            throw std::runtime_error(
                "mapSubGraphElementsAndConstraintsToOriginal: "
                "subgraph edge has no mapping to original graph");
        }

        auto constraint = m_constraintEdgeMap.get(originalEdge.value());

        if (constraint.has_value()) {
            result.m_constraintEdgeMap.set(
                resultEdge.value(), constraint.value().get());
        }

        // Propagate virtual edge status so that child ConstraintGraphs
        // retain knowledge of which edges are virtual from ancestor splits.
        if (m_virtualEdges.contains(originalEdge.value())) {
            result.m_virtualEdges.insert(resultEdge.value());
        }
    }

    // Look up where the original separator nodes ended up in the result
    // graph by following the chain: original -> subgraph local -> result.
    auto localSepA = subGraph.localNodeId(originalSepNodeA).value();
    auto localSepB = subGraph.localNodeId(originalSepNodeB).value();

    auto resultSepA = subLocalToResultNode.at(localSepA);
    auto resultSepB = subLocalToResultNode.at(localSepB);

    return SeparationGraphInfo {
        .separtionGraph = std::move(result),
        .originalToNewSepNodeA = { originalSepNodeA, resultSepA },
        .originalToNewSepNodeB = { originalSepNodeB, resultSepB },
    };
}

std::tuple<SeparationGraphInfo, SeparationGraphInfo>
ConstraintGraph::getSeparatingGraphs()
{
    // The function assumes that there is only one pair
    if (const auto& separationPairs
        = MathUtils::findFirstSeparationPair(m_constraintGraph);
        separationPairs.has_value()) {

        const auto& sepPair = separationPairs.value();
        std::vector<NodeIdType> splittingNodes { sepPair.a, sepPair.b };

        auto splitNodeSets
            = MathUtils::getSplitNodeSetsWithDuplicatedSeparators(
                m_constraintGraph, splittingNodes);

        // S-tree is a binary tree
        if (splittingNodes.size() > 2) {
            throw std::runtime_error(
                "getSeparatingGraphs function expects the separation nodes to "
                "create two subgraphs, but more is created.");
        }

        std::vector<MathUtils::SubGraph<Graph>> separationGraphs {};

        for (const auto& splitNodeSet : splitNodeSets) {
            separationGraphs.push_back(MathUtils::SubGraph<Graph>::extract(
                m_constraintGraph, splitNodeSet));
        }

        // Per the deficit/s-tree decomposition (Lemma 3.8): when a real
        // edge exists between the separation pair, it must appear in exactly
        // one separating graph.  The side that already contains virtual
        // edges from ancestor splits should keep the real edge (maximum
        // anchoring for the post-order solve).  The other side will later
        // receive a new virtual edge from the analysis() step.
        if (m_constraintGraph.hasEdgeBetween(sepPair.a, sepPair.b)) {
            auto edgeBetweenSep
                = m_constraintGraph.getEdgeBetween(sepPair.a, sepPair.b)
                      .value();

            // Only act on real edges; if the edge between the separator
            // pair is itself virtual, both subgraphs may keep their copy.
            if (!m_virtualEdges.contains(edgeBetweenSep)) {
                auto removeRealEdge
                    = [&sepPair](MathUtils::SubGraph<Graph>& graph) {
                          const auto& localSepNodeA
                              = graph.localNodeId(sepPair.a).value();
                          const auto& localSepNodeB
                              = graph.localNodeId(sepPair.b).value();
                          graph.removeEdge(
                              graph.getEdgeBetween(localSepNodeA, localSepNodeB)
                                  .value());
                      };

                // Count how many virtual edges fall into each side.
                int virtualCountSide0 = 0;
                int virtualCountSide1 = 0;
                for (const auto& vEdge : m_virtualEdges) {
                    const auto& [src, tgt]
                        = m_constraintGraph.getEndpoints(vEdge);
                    if (splitNodeSets[0].contains(src)
                        && splitNodeSets[0].contains(tgt)) {
                        ++virtualCountSide0;
                    } else if (splitNodeSets[1].contains(src)
                        && splitNodeSets[1].contains(tgt)) {
                        ++virtualCountSide1;
                    }
                }

                // Remove the real edge from the side WITHOUT virtual
                // edges so that the side WITH virtual edges keeps it
                // for better anchoring during post-order solving.
                // When neither side has virtual edges (first split),
                // default to removing from side 1.
                if (virtualCountSide1 > virtualCountSide0) {
                    removeRealEdge(separationGraphs[0]);
                } else {
                    removeRealEdge(separationGraphs[1]);
                }
            }
        }

        auto g1 = mapSubGraphElementsAndConstraintsToOriginal(
            separationGraphs[0], sepPair.a, sepPair.b);
        auto g2 = mapSubGraphElementsAndConstraintsToOriginal(
            separationGraphs[1], sepPair.a, sepPair.b);

        return std::tuple { std::move(g1), std::move(g2) };
    }

    throw std::runtime_error(
        "Tried to split graph that has no separation pairs");
}

int ConstraintGraph::numberOfSolvedElements() const
{
    return static_cast<int>(std::ranges::count_if(m_elementNodeMap,
        &Element::isElementSet, [](const auto& pair) { return pair.second; }));
}

}
