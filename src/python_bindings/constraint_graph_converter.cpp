#include "constraint_graph_converter.hpp"
#include <memory>
#include <vector>
#include <unordered_map>

namespace ConstraintGraphConverter {

std::shared_ptr<Gcs::Element> convertElement(
    const SimpleCG::SimpleElement& simpleElement)
{
    switch (simpleElement.type) {
    case SimpleCG::ElementType::Point: {
        Gcs::Point point(simpleElement.data[0], simpleElement.data[1]);
        return std::make_shared<Gcs::Element>(point);
    }
    case SimpleCG::ElementType::FixedRadiusCircle: {
        Gcs::FixedRadiusCircle circle(simpleElement.data[0], // x
            simpleElement.data[1], // y
            simpleElement.data[2] // radius
        );
        return std::make_shared<Gcs::Element>(circle);
    }
    case SimpleCG::ElementType::Line: {
        Gcs::Line line(simpleElement.data[0], // r0_x
            simpleElement.data[1], // r0_y
            simpleElement.data[2], // v_x
            simpleElement.data[3] // v_y
        );
        return std::make_shared<Gcs::Element>(line);
    }
    default:
        throw std::invalid_argument("Unknown element type");
    }
}

std::shared_ptr<Gcs::Constraint> convertConstraint(
    const SimpleCG::SimpleConstraint& simpleConstraint)
{
    switch (simpleConstraint.type) {
    case SimpleCG::ConstraintType::Distance: {
        Gcs::DistanceConstraint distanceConstraint(simpleConstraint.value);
        return std::make_shared<Gcs::Constraint>(distanceConstraint);
    }
    case SimpleCG::ConstraintType::Tangency: {
        Gcs::TangencyConstraint tangencyConstraint(simpleConstraint.value);
        return std::make_shared<Gcs::Constraint>(tangencyConstraint);
    }
    default:
        throw std::invalid_argument("Unknown constraint type");
    }
}

Gcs::ConstraintGraph convertToConstraintGraph(
    const SimpleCG::SimpleConstraintGraph& simpleGraph)
{
    Gcs::ConstraintGraph constraintGraph;

    // Store node mapping from simple graph node IDs to ConstraintGraph nodes
    std::unordered_map<int, Gcs::ConstraintGraph::NodeType*> nodeMapping;

    // Convert and add all nodes
    const auto& simpleNodes = simpleGraph.getNodes();
    for (int i = 0; i < static_cast<int>(simpleNodes.size()); ++i) {
        auto convertedElement = convertElement(simpleNodes[i]);
        auto& addedNode = constraintGraph.addNode(convertedElement);
        nodeMapping[i] = &addedNode;
    }

    // Convert and add all edges
    const auto& simpleEdges = simpleGraph.getEdges();
    for (const auto& simpleEdge : simpleEdges) {
        // Get the nodes for this edge
        auto node1It = nodeMapping.find(simpleEdge.nodeId1);
        auto node2It = nodeMapping.find(simpleEdge.nodeId2);

        if (node1It == nodeMapping.end() || node2It == nodeMapping.end()) {
            throw std::runtime_error("Invalid node IDs in edge: "
                + std::to_string(simpleEdge.nodeId1) + ", "
                + std::to_string(simpleEdge.nodeId2));
        }

        // Convert the constraint
        auto convertedConstraint = convertConstraint(simpleEdge.constraint);

        // Add the edge
        constraintGraph.addEdge(
            *node1It->second, *node2It->second, convertedConstraint);
    }

    return constraintGraph;
}

SimpleCG::SimpleElement convertElementBack(
    const std::shared_ptr<Gcs::Element>& element)
{
    SimpleCG::SimpleElement simpleElement;

    element->visitElement([&simpleElement](const auto& e) {
        if constexpr (std::is_same_v<std::decay_t<decltype(e)>, Gcs::Point>) {
            simpleElement = SimpleCG::SimpleElement::createPoint(e.x, e.y);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(e)>,
                                 Gcs::FixedRadiusCircle>) {
            simpleElement = SimpleCG::SimpleElement::createCircle(
                e.x, e.y, e.fixed_radius);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(e)>,
                                 Gcs::Line>) {
            simpleElement
                = SimpleCG::SimpleElement::createLine(e.r0_x, e.r0_y, e.v_x, e.v_y);
        }
    });

    return simpleElement;
}

SimpleCG::SimpleConstraint convertConstraintBack(
    const std::shared_ptr<Gcs::Constraint>& constraint)
{
    SimpleCG::SimpleConstraint simpleConstraint;

    constraint->visitConsraint([&simpleConstraint](const auto& c) {
        if constexpr (std::is_same_v<std::decay_t<decltype(c)>,
                          Gcs::DistanceConstraint>) {
            simpleConstraint
                = SimpleCG::SimpleConstraint::createDistance(c.distance);
        } else if constexpr (std::is_same_v<std::decay_t<decltype(c)>,
                                 Gcs::TangencyConstraint>) {
            simpleConstraint
                = SimpleCG::SimpleConstraint::createTangency(c.angle);
        }
    });

    return simpleConstraint;
}

SimpleCG::SimpleConstraintGraph convertFromConstraintGraph(
    const Gcs::ConstraintGraph& constraintGraph)
{
    SimpleCG::SimpleConstraintGraph simpleGraph;

    // Map from ConstraintGraph nodes to simple graph node IDs
    std::unordered_map<const Gcs::ConstraintGraph::NodeType*, int> nodeMapping;

    // Convert all nodes
    const auto& nodes = constraintGraph.getNodes();
    std::vector<const Gcs::ConstraintGraph::NodeType*> nodePointers;

    for (const auto& node : nodes) {
        auto element = node.getStoredObj();
        auto simpleElement = convertElementBack(element);
        int nodeId = simpleGraph.addNode(simpleElement);
        nodeMapping[&node] = nodeId;
        nodePointers.push_back(&node);
    }

    // Convert all edges by checking between every pair of nodes
    for (size_t i = 0; i < nodePointers.size(); ++i) {
        for (size_t j = i + 1; j < nodePointers.size(); ++j) {
            auto edgeOpt = constraintGraph.getEdgeBetweenNodes(
                *nodePointers[i], *nodePointers[j]);

            if (edgeOpt.has_value()) {
                auto& edge = edgeOpt.value();
                auto constraint = edge.getStoredObj();
                auto simpleConstraint = convertConstraintBack(constraint);

                auto it1 = nodeMapping.find(nodePointers[i]);
                auto it2 = nodeMapping.find(nodePointers[j]);

                if (it1 != nodeMapping.end() && it2 != nodeMapping.end()) {
                    simpleGraph.addEdge(it1->second, it2->second, simpleConstraint);
                }
            }
        }
    }

    return simpleGraph;
}

} // namespace ConstraintGraphConverter
