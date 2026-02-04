#ifndef GCS_DATA_STRUCTURES_HPP
#define GCS_DATA_STRUCTURES_HPP

#include "constraints.hpp"
#include "elements.hpp"

#include <cstddef>
#include <expected>
#include <memory>
#include <ranges>
#include <span>
#include <structures/graph.hpp>
#include <structures/graph_errors.hpp>
#include <structures/property_map.hpp>
#include <structures/separation_pairs.hpp>
#include <structures/simple_graph.hpp>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Gcs {

enum class ConstraintGraphError { OK, NodeNotFound, EdgeNotFound };

struct SeparationGraphInfo;

class ConstraintGraph final {
public:
    using Graph = MathUtils::SimpleGraph;
    using NodeIdType = Graph::NodeIdType;
    using EdgeIdType = Graph::EdgeIdType;

    std::tuple<SeparationGraphInfo, SeparationGraphInfo> getSeparatingGraphs();

    ConstraintGraphError addElement(
        NodeIdType node, std::shared_ptr<Element> element);
    ConstraintGraphError addConstraint(
        Graph::EdgeIdType edge, std::shared_ptr<Constraint> constraint);

    std::expected<EdgeIdType, ConstraintGraphError> getEdgeBetween(
        NodeIdType s, NodeIdType t)
    {
        const auto& edge = m_constraintGraph.getEdgeBetween(s, t);
        if (!edge.has_value()) {
            return std::unexpected(ConstraintGraphError::EdgeNotFound);
        }

        return edge.value();
    }

    EdgeIdType addVirtualEdge(NodeIdType s, NodeIdType t);

    ConstraintGraphError removeVirtualEdge(EdgeIdType virtualEdge);

    ConstraintGraphError removeElement(NodeIdType node);

    ConstraintGraphError removeConstraintEdge(EdgeIdType edge);

    std::shared_ptr<Element> getElement(NodeIdType node) const;

    std::shared_ptr<Constraint> getConstraintForEdge(EdgeIdType edge) const;
    std::shared_ptr<Constraint> getConstraintBetweenNodes(
        NodeIdType s, NodeIdType t) const
    {
        const auto& edge = m_constraintGraph.getEdgeBetween(s, t).value();
        return m_constraintEdgeMap.get(edge).value();
    }

    const MathUtils::NodePropertyMap<std::shared_ptr<Element>>&
    getElementMap() const
    {
        return m_elementNodeMap;
    }

    const MathUtils::EdgePropertyMap<std::shared_ptr<Constraint>>&
    getConstraintMap() const
    {
        return m_constraintEdgeMap;
    }

    bool hasVirtualEdge() const { return !m_virtualEdges.empty(); }

    /**
     * @brief Check whether a specific edge is virtual.
     * @param edge The edge to check.
     * @return @c true if @p edge is in the virtual edge set.
     */
    bool isVirtualEdge(EdgeIdType edge) const
    {
        return m_virtualEdges.contains(edge);
    }

    /**
     * @brief Get the set of all virtual edges.
     * @return Const reference to the virtual edge set.
     */
    const std::unordered_set<EdgeIdType>& getVirtualEdges() const
    {
        return m_virtualEdges;
    }

    std::size_t nodeCount() const { return m_constraintGraph.nodeCount(); }

    std::size_t edgeCount() const { return m_constraintGraph.edgeCount(); }

    int numberOfSolvedElements() const;

    int getDeficit()
    {
        return (2 * m_constraintGraph.nodeCount() - 3)
            - m_constraintGraph.edgeCount();
    }

    Graph& getGraph() { return m_constraintGraph; }
    const Graph& getGraph() const { return m_constraintGraph; }

    bool isTriconnected()
    {
        return MathUtils::isTriconnected(m_constraintGraph).value();
    };

    std::vector<std::shared_ptr<Element>> getElements() const
    {
        auto values = m_elementNodeMap | std::views::values;
        return { values.begin(), values.end() };
    }

    std::vector<std::shared_ptr<Constraint>> getConstraints() const
    {
        auto values = m_constraintEdgeMap | std::views::values;
        return { values.begin(), values.end() };
    }

private:
    SeparationGraphInfo mapSubGraphElementsAndConstraintsToOriginal(
        const MathUtils::SubGraph<Graph>& subGraph, NodeIdType originalSepNodeA,
        NodeIdType originalSepNodeB);

    Graph m_constraintGraph;
    MathUtils::NodePropertyMap<std::shared_ptr<Element>> m_elementNodeMap;
    MathUtils::EdgePropertyMap<std::shared_ptr<Constraint>> m_constraintEdgeMap;
    std::unordered_set<EdgeIdType> m_virtualEdges;
};

struct SeparationGraphInfo {
    ConstraintGraph separtionGraph;
    std::pair<ConstraintGraph::NodeIdType, ConstraintGraph::NodeIdType>
        originalToNewSepNodeA;
    std::pair<ConstraintGraph::NodeIdType, ConstraintGraph::NodeIdType>
        originalToNewSepNodeB;
};

} // namespace Gcs

#endif
