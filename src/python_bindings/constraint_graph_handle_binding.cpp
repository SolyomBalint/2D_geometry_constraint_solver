#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

#include "constraint_graph_handle.hpp"

namespace nb = nanobind;

NB_MODULE(constraint_graph_handle_binding, m)
{
    m.doc() = "Opaque handle for 2D Geometry Constraint Graph - Simplified Python Bindings";

    // Bind NodePosition struct
    nb::class_<GcsBinding::NodePosition>(m, "NodePosition")
        .def(nb::init<int, const std::vector<double>&>(),
            nb::arg("nodeId"), nb::arg("data"))
        .def_rw("nodeId", &GcsBinding::NodePosition::nodeId,
            "Node ID")
        .def_rw("data", &GcsBinding::NodePosition::data,
            "Position data - Point: [x, y], Circle: [x, y, r], Line: [r0_x, r0_y, v_x, v_y]")
        .def("__repr__", [](const GcsBinding::NodePosition& pos) {
            std::ostringstream oss;
            oss << "NodePosition(nodeId=" << pos.nodeId << ", data=[";
            for (size_t i = 0; i < pos.data.size(); ++i) {
                oss << pos.data[i];
                if (i < pos.data.size() - 1) oss << ", ";
            }
            oss << "])";
            return oss.str();
        });

    // Bind NodeInfo struct (for visualization)
    nb::class_<GcsBinding::NodeInfo>(m, "NodeInfo")
        .def(nb::init<int, const std::string&, const std::vector<double>&>(),
            nb::arg("nodeId"), nb::arg("type"), nb::arg("data"))
        .def_rw("nodeId", &GcsBinding::NodeInfo::nodeId,
            "Node ID")
        .def_rw("type", &GcsBinding::NodeInfo::type,
            "Element type (Point, Circle, Line)")
        .def_rw("data", &GcsBinding::NodeInfo::data,
            "Element data - Point: [x, y], Circle: [x, y, r], Line: [r0_x, r0_y, v_x, v_y]")
        .def("__repr__", [](const GcsBinding::NodeInfo& info) {
            std::ostringstream oss;
            oss << "NodeInfo(nodeId=" << info.nodeId << ", type='" << info.type << "', data=[";
            for (size_t i = 0; i < info.data.size(); ++i) {
                oss << info.data[i];
                if (i < info.data.size() - 1) oss << ", ";
            }
            oss << "])";
            return oss.str();
        });

    // Bind EdgeInfo struct (for visualization)
    nb::class_<GcsBinding::EdgeInfo>(m, "EdgeInfo")
        .def(nb::init<int, int, const std::string&, double>(),
            nb::arg("nodeId1"), nb::arg("nodeId2"), nb::arg("type"), nb::arg("value"))
        .def_rw("nodeId1", &GcsBinding::EdgeInfo::nodeId1,
            "First node ID")
        .def_rw("nodeId2", &GcsBinding::EdgeInfo::nodeId2,
            "Second node ID")
        .def_rw("type", &GcsBinding::EdgeInfo::type,
            "Constraint type (Distance, Tangency)")
        .def_rw("value", &GcsBinding::EdgeInfo::value,
            "Constraint value")
        .def("__repr__", [](const GcsBinding::EdgeInfo& info) {
            std::ostringstream oss;
            oss << "EdgeInfo(nodeId1=" << info.nodeId1 << ", nodeId2=" << info.nodeId2
                << ", type='" << info.type << "', value=" << info.value << ")";
            return oss.str();
        });

    // Bind SubgraphInfo struct (for decomposition)
    nb::class_<GcsBinding::SubgraphInfo>(m, "SubgraphInfo")
        .def(nb::init<>())
        .def(nb::init<const std::vector<GcsBinding::NodeInfo>&, const std::vector<GcsBinding::EdgeInfo>&>(),
            nb::arg("nodes"), nb::arg("edges"))
        .def_rw("nodes", &GcsBinding::SubgraphInfo::nodes,
            "List of NodeInfo for nodes in this subgraph")
        .def_rw("edges", &GcsBinding::SubgraphInfo::edges,
            "List of EdgeInfo for edges in this subgraph")
        .def("__repr__", [](const GcsBinding::SubgraphInfo& info) {
            std::ostringstream oss;
            oss << "SubgraphInfo(nodes=" << info.nodes.size()
                << ", edges=" << info.edges.size() << ")";
            return oss.str();
        });

    // Bind ConstraintGraphHandle class
    nb::class_<GcsBinding::ConstraintGraphHandle>(m, "ConstraintGraphHandle",
        "Opaque handle for ConstraintGraph with simplified Python-friendly API")
        .def(nb::init<>(), "Create a new empty constraint graph")

        // Node addition methods
        .def("addPoint", &GcsBinding::ConstraintGraphHandle::addPoint,
            nb::arg("canvasX"), nb::arg("canvasY"),
            "Add a Point element to the graph. Returns node ID.\n\n"
            "Args:\n"
            "    canvasX: X coordinate in canvas space\n"
            "    canvasY: Y coordinate in canvas space (Y+ is down)\n"
            "Returns:\n"
            "    Node ID for this element")

        .def("addCircle", &GcsBinding::ConstraintGraphHandle::addCircle,
            nb::arg("canvasX"), nb::arg("canvasY"), nb::arg("radius"),
            "Add a FixedRadiusCircle element to the graph. Returns node ID.\n\n"
            "Args:\n"
            "    canvasX: X coordinate of circle center in canvas space\n"
            "    canvasY: Y coordinate of circle center in canvas space (Y+ is down)\n"
            "    radius: Circle radius\n"
            "Returns:\n"
            "    Node ID for this element")

        .def("addLine", &GcsBinding::ConstraintGraphHandle::addLine,
            nb::arg("r0X"), nb::arg("r0Y"), nb::arg("vX"), nb::arg("vY"),
            "Add a Line element to the graph. Returns node ID.\n\n"
            "Args:\n"
            "    r0X: X coordinate of line reference point in canvas space\n"
            "    r0Y: Y coordinate of line reference point in canvas space (Y+ is down)\n"
            "    vX: X component of direction vector\n"
            "    vY: Y component of direction vector (in canvas space)\n"
            "Returns:\n"
            "    Node ID for this element")

        // Constraint addition methods
        .def("addDistanceConstraint", &GcsBinding::ConstraintGraphHandle::addDistanceConstraint,
            nb::arg("nodeId1"), nb::arg("nodeId2"), nb::arg("distance"),
            "Add a distance constraint between two nodes.\n\n"
            "Args:\n"
            "    nodeId1: First node ID\n"
            "    nodeId2: Second node ID\n"
            "    distance: Desired distance between elements\n"
            "Returns:\n"
            "    True if constraint was added successfully")

        .def("addTangencyConstraint", &GcsBinding::ConstraintGraphHandle::addTangencyConstraint,
            nb::arg("nodeId1"), nb::arg("nodeId2"), nb::arg("angle"),
            "Add a tangency constraint between two nodes.\n\n"
            "Args:\n"
            "    nodeId1: First node ID\n"
            "    nodeId2: Second node ID\n"
            "    angle: Tangency angle\n"
            "Returns:\n"
            "    True if constraint was added successfully")

        // Solving and analysis methods
        .def("solve", &GcsBinding::ConstraintGraphHandle::solve,
            "Solve the constraint system using the full GCS pipeline.\n\n"
            "Returns:\n"
            "    List of NodePosition with solved coordinates (in canvas space)")

        .def("isWellConstrained", &GcsBinding::ConstraintGraphHandle::isWellConstrained,
            "Check if the constraint graph is well-constrained using Laman's theorem.\n\n"
            "Returns:\n"
            "    True if the graph is well-constrained (edges = 2*nodes - 3)")

        .def("getNodeCount", &GcsBinding::ConstraintGraphHandle::getNodeCount,
            "Get the number of nodes in the graph.\n\n"
            "Returns:\n"
            "    Node count")

        .def("getEdgeCount", &GcsBinding::ConstraintGraphHandle::getEdgeCount,
            "Get the number of edges (constraints) in the graph.\n\n"
            "Returns:\n"
            "    Edge count")

        .def("getDecompositionInfo", &GcsBinding::ConstraintGraphHandle::getDecompositionInfo,
            "Decompose the graph into subgraphs.\n\n"
            "Returns:\n"
            "    List of subgraph sizes (number of nodes in each subgraph)")

        .def("clear", &GcsBinding::ConstraintGraphHandle::clear,
            "Clear all nodes and edges from the graph")

        .def("getNodeInfo", &GcsBinding::ConstraintGraphHandle::getNodeInfo,
            nb::arg("nodeId"),
            "Get information about a specific node.\n\n"
            "Args:\n"
            "    nodeId: Node ID to query\n"
            "Returns:\n"
            "    String description of the node")

        .def("getNodes", &GcsBinding::ConstraintGraphHandle::getNodes,
            "Get all nodes in the graph for visualization.\n\n"
            "Returns:\n"
            "    List of NodeInfo with nodeId, type, and data for each node")

        .def("getEdges", &GcsBinding::ConstraintGraphHandle::getEdges,
            "Get all edges/constraints in the graph for visualization.\n\n"
            "Returns:\n"
            "    List of EdgeInfo with nodeId1, nodeId2, type, and value for each edge")

        .def("getDecomposedSubgraphs", &GcsBinding::ConstraintGraphHandle::getDecomposedSubgraphs,
            "Decompose the graph into subgraphs and return full structure.\n\n"
            "Returns:\n"
            "    List of SubgraphInfo, each containing nodes and edges for that subgraph")

        .def("__repr__", [](const GcsBinding::ConstraintGraphHandle& handle) {
            return "ConstraintGraphHandle(nodes=" + std::to_string(handle.getNodeCount()) +
                   ", edges=" + std::to_string(handle.getEdgeCount()) + ")";
        });
}
