#include "simple_constraint_graph.hpp"
#include <stdexcept>
#include <sstream>

namespace SimpleCG {

std::string SimpleElement::toString() const
{
    std::ostringstream oss;
    switch (type) {
    case ElementType::Point:
        oss << "Point(x: " << data[0] << ", y: " << data[1] << ")";
        break;
    case ElementType::FixedRadiusCircle:
        oss << "FixedRadiusCircle(x: " << data[0] << ", y: " << data[1]
            << ", radius: " << data[2] << ")";
        break;
    case ElementType::Line:
        oss << "Line(r0: (" << data[0] << ", " << data[1] << "), v: ("
            << data[2] << ", " << data[3] << "))";
        break;
    }
    return oss.str();
}

std::string SimpleConstraint::toString() const
{
    std::ostringstream oss;
    switch (type) {
    case ConstraintType::Distance:
        oss << "DistanceConstraint(distance: " << value << ")";
        break;
    case ConstraintType::Tangency:
        oss << "TangencyConstraint(angle: " << value << ")";
        break;
    }
    return oss.str();
}

int SimpleConstraintGraph::addNode(const SimpleElement& element)
{
    nodes.push_back(element);
    return static_cast<int>(nodes.size() - 1);
}

const SimpleElement& SimpleConstraintGraph::getNode(int nodeId) const
{
    if (nodeId < 0 || nodeId >= static_cast<int>(nodes.size())) {
        throw std::out_of_range("Invalid node ID: " + std::to_string(nodeId));
    }
    return nodes[nodeId];
}

int SimpleConstraintGraph::addEdge(
    int nodeId1, int nodeId2, const SimpleConstraint& constraint)
{
    // Validate node IDs
    if (nodeId1 < 0 || nodeId1 >= static_cast<int>(nodes.size()) || nodeId2 < 0
        || nodeId2 >= static_cast<int>(nodes.size())) {
        throw std::out_of_range("Invalid node IDs for edge creation");
    }

    edges.emplace_back(nodeId1, nodeId2, constraint);
    return static_cast<int>(edges.size() - 1);
}

const SimpleEdge& SimpleConstraintGraph::getEdge(int edgeId) const
{
    if (edgeId < 0 || edgeId >= static_cast<int>(edges.size())) {
        throw std::out_of_range("Invalid edge ID: " + std::to_string(edgeId));
    }
    return edges[edgeId];
}

void SimpleConstraintGraph::clear()
{
    nodes.clear();
    edges.clear();
}

std::string SimpleConstraintGraph::toString() const
{
    std::ostringstream oss;
    oss << "SimpleConstraintGraph(nodes: " << nodes.size()
        << ", edges: " << edges.size() << ")";
    return oss.str();
}

} // namespace SimpleCG
