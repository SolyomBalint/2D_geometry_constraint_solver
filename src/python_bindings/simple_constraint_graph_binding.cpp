#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

#include "simple_constraint_graph.hpp"
#include "constraint_graph_converter.hpp"
#include "../constraint_solver/geometric_constraint_system.hpp"

namespace nb = nanobind;

NB_MODULE(simple_constraint_graph_binding, m)
{
    m.doc() = "Simple 2D Geometry Constraint Graph Python Bindings";

    // Bind ElementType enum
    nb::enum_<SimpleCG::ElementType>(m, "ElementType")
        .value("Point", SimpleCG::ElementType::Point)
        .value("FixedRadiusCircle", SimpleCG::ElementType::FixedRadiusCircle)
        .value("Line", SimpleCG::ElementType::Line)
        .export_values();

    // Bind ConstraintType enum
    nb::enum_<SimpleCG::ConstraintType>(m, "ConstraintType")
        .value("Distance", SimpleCG::ConstraintType::Distance)
        .value("Tangency", SimpleCG::ConstraintType::Tangency)
        .export_values();

    // Bind SimpleElement
    nb::class_<SimpleCG::SimpleElement>(m, "SimpleElement")
        .def(nb::init<>())
        .def(nb::init<SimpleCG::ElementType, double, double, double, double>(),
            nb::arg("type"), nb::arg("d1"), nb::arg("d2"), nb::arg("d3") = 0.0,
            nb::arg("d4") = 0.0)
        .def_rw("type", &SimpleCG::SimpleElement::type)
        .def_prop_rw(
            "data",
            [](const SimpleCG::SimpleElement& self) -> std::vector<double> {
                return std::vector<double>(self.data, self.data + 4);
            },
            [](SimpleCG::SimpleElement& self, const std::vector<double>& data) {
                if (data.size() > 4) {
                    throw std::invalid_argument(
                        "Data array cannot have more than 4 elements");
                }
                for (size_t i = 0; i < std::min(data.size(), size_t(4)); ++i) {
                    self.data[i] = data[i];
                }
            })
        .def_static("createPoint", &SimpleCG::SimpleElement::createPoint,
            nb::arg("x"), nb::arg("y"), "Create a Point element")
        .def_static("createCircle", &SimpleCG::SimpleElement::createCircle,
            nb::arg("x"), nb::arg("y"), nb::arg("radius"),
            "Create a FixedRadiusCircle element")
        .def_static("createLine", &SimpleCG::SimpleElement::createLine,
            nb::arg("r0_x"), nb::arg("r0_y"), nb::arg("v_x"), nb::arg("v_y"),
            "Create a Line element")
        .def("toString", &SimpleCG::SimpleElement::toString)
        .def("__repr__", &SimpleCG::SimpleElement::toString);

    // Bind SimpleConstraint
    nb::class_<SimpleCG::SimpleConstraint>(m, "SimpleConstraint")
        .def(nb::init<>())
        .def(nb::init<SimpleCG::ConstraintType, double>(), nb::arg("type"),
            nb::arg("value"))
        .def_rw("type", &SimpleCG::SimpleConstraint::type)
        .def_rw("value", &SimpleCG::SimpleConstraint::value)
        .def_static("createDistance",
            &SimpleCG::SimpleConstraint::createDistance, nb::arg("distance"),
            "Create a Distance constraint")
        .def_static("createTangency",
            &SimpleCG::SimpleConstraint::createTangency, nb::arg("angle"),
            "Create a Tangency constraint")
        .def("toString", &SimpleCG::SimpleConstraint::toString)
        .def("__repr__", &SimpleCG::SimpleConstraint::toString);

    // Bind SimpleEdge
    nb::class_<SimpleCG::SimpleEdge>(m, "SimpleEdge")
        .def(nb::init<>())
        .def(nb::init<int, int, const SimpleCG::SimpleConstraint&>(),
            nb::arg("nodeId1"), nb::arg("nodeId2"), nb::arg("constraint"))
        .def_rw("nodeId1", &SimpleCG::SimpleEdge::nodeId1)
        .def_rw("nodeId2", &SimpleCG::SimpleEdge::nodeId2)
        .def_rw("constraint", &SimpleCG::SimpleEdge::constraint)
        .def("__repr__", [](const SimpleCG::SimpleEdge& e) -> std::string {
            return "SimpleEdge(nodeId1=" + std::to_string(e.nodeId1)
                + ", nodeId2=" + std::to_string(e.nodeId2)
                + ", constraint=" + e.constraint.toString() + ")";
        });

    // Bind SimpleConstraintGraph
    nb::class_<SimpleCG::SimpleConstraintGraph>(m, "SimpleConstraintGraph")
        .def(nb::init<>())
        .def("addNode", &SimpleCG::SimpleConstraintGraph::addNode,
            nb::arg("element"), "Add an element as a node and return its ID")
        .def("addEdge", &SimpleCG::SimpleConstraintGraph::addEdge,
            nb::arg("nodeId1"), nb::arg("nodeId2"), nb::arg("constraint"),
            "Add a constraint as an edge between two nodes and return its ID")
        .def("getNodeCount", &SimpleCG::SimpleConstraintGraph::getNodeCount)
        .def("getEdgeCount", &SimpleCG::SimpleConstraintGraph::getEdgeCount)
        .def("getNode", &SimpleCG::SimpleConstraintGraph::getNode,
            nb::arg("nodeId"), "Get node by ID")
        .def("getEdge", &SimpleCG::SimpleConstraintGraph::getEdge,
            nb::arg("edgeId"), "Get edge by ID")
        .def("getNodes", &SimpleCG::SimpleConstraintGraph::getNodes,
            nb::rv_policy::reference_internal, "Get all nodes")
        .def("getEdges", &SimpleCG::SimpleConstraintGraph::getEdges,
            nb::rv_policy::reference_internal, "Get all edges")
        .def("clear", &SimpleCG::SimpleConstraintGraph::clear)
        .def("toString", &SimpleCG::SimpleConstraintGraph::toString)
        .def("__repr__", &SimpleCG::SimpleConstraintGraph::toString);

    // Convenience function to solve and return updated positions
    m.def(
        "solveSimpleConstraintGraph",
        [](const SimpleCG::SimpleConstraintGraph& simpleGraph)
            -> std::vector<std::vector<double>> {
            // Convert to ConstraintGraph
            auto constraintGraph
                = ConstraintGraphConverter::convertToConstraintGraph(
                    simpleGraph);

            // Solve the constraint graph
            Gcs::GeometricConstraintSystem solver;
            solver.solveGcsViaPipeline(constraintGraph);

            // Extract solved positions
            std::vector<std::vector<double>> positions;
            auto nodes = constraintGraph.getNodes();

            for (const auto& node : nodes) {
                auto element = node.getStoredObj();
                std::vector<double> nodeData;

                // Use visitor pattern to extract position data
                element->visitElement([&nodeData](const auto& e) {
                    if constexpr (std::is_same_v<std::decay_t<decltype(e)>,
                                      Gcs::Point>) {
                        nodeData = { e.x, e.y };
                    } else if constexpr (std::is_same_v<
                                             std::decay_t<decltype(e)>,
                                             Gcs::FixedRadiusCircle>) {
                        nodeData = { e.x, e.y, e.fixed_radius };
                    } else if constexpr (std::is_same_v<
                                             std::decay_t<decltype(e)>,
                                             Gcs::Line>) {
                        nodeData = { e.r0_x, e.r0_y, e.v_x, e.v_y };
                    }
                });

                positions.push_back(nodeData);
            }

            return positions;
        },
        nb::arg("simpleGraph"),
        "Convert, solve and return solved positions from a "
        "SimpleConstraintGraph");

    // Function to check if a constraint graph is well-constrained
    m.def(
        "isWellConstrained",
        [](const SimpleCG::SimpleConstraintGraph& simpleGraph) -> bool {
            auto constraintGraph
                = ConstraintGraphConverter::convertToConstraintGraph(
                    simpleGraph);
            return Gcs::defaultDetectorFunc(constraintGraph);
        },
        nb::arg("simpleGraph"),
        "Check if a SimpleConstraintGraph is well-constrained using Laman's "
        "theorem");

    // Function to get decomposition information
    m.def(
        "getDecompositionInfo",
        [](const SimpleCG::SimpleConstraintGraph& simpleGraph)
            -> std::vector<int> {
            auto constraintGraph
                = ConstraintGraphConverter::convertToConstraintGraph(
                    simpleGraph);
            auto subgraphs = Gcs::defaultDecompositorFunc(constraintGraph);

            std::vector<int> subgraphSizes;
            for (const auto& subgraph : subgraphs) {
                subgraphSizes.push_back(
                    static_cast<int>(subgraph.getNodeCount()));
            }
            return subgraphSizes;
        },
        nb::arg("simpleGraph"),
        "Get decomposition information (subgraph sizes) for a "
        "SimpleConstraintGraph");

    // Function to decompose constraint graph into subgraphs
    m.def(
        "decomposeConstraintGraph",
        [](const SimpleCG::SimpleConstraintGraph& simpleGraph)
            -> std::vector<SimpleCG::SimpleConstraintGraph> {
            // Convert to ConstraintGraph
            auto constraintGraph
                = ConstraintGraphConverter::convertToConstraintGraph(
                    simpleGraph);

            // Decompose the graph
            auto subgraphs = Gcs::defaultDecompositorFunc(constraintGraph);

            // Convert back to SimpleConstraintGraphs
            std::vector<SimpleCG::SimpleConstraintGraph> simpleSubgraphs;
            for (auto& subgraph : subgraphs) {
                simpleSubgraphs.push_back(
                    ConstraintGraphConverter::convertFromConstraintGraph(
                        subgraph));
            }

            return simpleSubgraphs;
        },
        nb::arg("simpleGraph"),
        "Decompose a SimpleConstraintGraph into subgraphs and return them as "
        "SimpleConstraintGraphs");
}
