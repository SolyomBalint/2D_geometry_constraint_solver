#ifndef CONSTRAINT_GRAPH_HANDLE_HPP
#define CONSTRAINT_GRAPH_HANDLE_HPP

#include "../constraint_solver/geometric_constraint_system.hpp"
#include <vector>
#include <unordered_map>
#include <string>

namespace GcsBinding {

/**
 * @brief Result structure for solved node positions
 */
struct NodePosition {
    int nodeId;
    std::vector<double> data; // Point: [x, y], Circle: [x, y, r], Line: [r0_x, r0_y, v_x, v_y]

    NodePosition(int id, const std::vector<double>& d)
        : nodeId(id), data(d) {}
};

/**
 * @brief Information about a node in the graph (for visualization)
 */
struct NodeInfo {
    int nodeId;
    std::string type;  // "Point", "Circle", "Line"
    std::vector<double> data;  // Same format as NodePosition

    NodeInfo(int id, const std::string& t, const std::vector<double>& d)
        : nodeId(id), type(t), data(d) {}
};

/**
 * @brief Information about an edge/constraint in the graph (for visualization)
 */
struct EdgeInfo {
    int nodeId1;
    int nodeId2;
    std::string type;  // "Distance", "Tangency"
    double value;

    EdgeInfo(int id1, int id2, const std::string& t, double v)
        : nodeId1(id1), nodeId2(id2), type(t), value(v) {}
};

/**
 * @brief Information about a single subgraph after decomposition
 */
struct SubgraphInfo {
    std::vector<NodeInfo> nodes;
    std::vector<EdgeInfo> edges;

    SubgraphInfo() = default;
    SubgraphInfo(const std::vector<NodeInfo>& n, const std::vector<EdgeInfo>& e)
        : nodes(n), edges(e) {}
};

/**
 * @brief Opaque handle for ConstraintGraph with Python-friendly API
 *
 * This class wraps the complex ConstraintGraph structure and provides a simple
 * interface suitable for Python bindings. It handles all coordinate transformations,
 * node ID management, and graph operations internally.
 */
class ConstraintGraphHandle {
private:
    Gcs::ConstraintGraph graph_;
    std::unordered_map<int, Gcs::ConstraintGraph::NodeType*> nodeIdMap_;
    int nextNodeId_ = 0;

    // Helper methods for coordinate transformation
    static double flipYForSolver(double canvasY) { return -canvasY; }
    static double flipYForCanvas(double solverY) { return -solverY; }

public:
    ConstraintGraphHandle() = default;

    // Non-copyable but movable
    ConstraintGraphHandle(const ConstraintGraphHandle&) = delete;
    ConstraintGraphHandle& operator=(const ConstraintGraphHandle&) = delete;
    ConstraintGraphHandle(ConstraintGraphHandle&&) noexcept = default;
    ConstraintGraphHandle& operator=(ConstraintGraphHandle&&) noexcept = default;

    /**
     * @brief Add a Point element to the graph
     * @param canvasX X coordinate in canvas space (unchanged)
     * @param canvasY Y coordinate in canvas space (Y+ is down)
     * @return Node ID for this element
     */
    int addPoint(double canvasX, double canvasY);

    /**
     * @brief Add a FixedRadiusCircle element to the graph
     * @param canvasX X coordinate of circle center in canvas space
     * @param canvasY Y coordinate of circle center in canvas space (Y+ is down)
     * @param radius Circle radius
     * @return Node ID for this element
     */
    int addCircle(double canvasX, double canvasY, double radius);

    /**
     * @brief Add a Line element to the graph
     * @param r0X X coordinate of line reference point in canvas space
     * @param r0Y Y coordinate of line reference point in canvas space (Y+ is down)
     * @param vX X component of direction vector
     * @param vY Y component of direction vector (in canvas space)
     * @return Node ID for this element
     */
    int addLine(double r0X, double r0Y, double vX, double vY);

    /**
     * @brief Add a distance constraint between two nodes
     * @param nodeId1 First node ID
     * @param nodeId2 Second node ID
     * @param distance Desired distance between elements
     * @return true if constraint was added successfully
     */
    bool addDistanceConstraint(int nodeId1, int nodeId2, double distance);

    /**
     * @brief Add a tangency constraint between two nodes
     * @param nodeId1 First node ID
     * @param nodeId2 Second node ID
     * @param angle Tangency angle
     * @return true if constraint was added successfully
     */
    bool addTangencyConstraint(int nodeId1, int nodeId2, double angle);

    /**
     * @brief Solve the constraint system using the full GCS pipeline
     * @return Vector of NodePosition with solved coordinates (in canvas space)
     */
    std::vector<NodePosition> solve();

    /**
     * @brief Check if the constraint graph is well-constrained using Laman's theorem
     * @return true if the graph is well-constrained (edges = 2*nodes - 3)
     */
    bool isWellConstrained() const;

    /**
     * @brief Get the number of nodes in the graph
     * @return Node count
     */
    int getNodeCount() const;

    /**
     * @brief Get the number of edges (constraints) in the graph
     * @return Edge count
     */
    int getEdgeCount() const;

    /**
     * @brief Decompose the graph into subgraphs
     * @return Vector of subgraph sizes (number of nodes in each subgraph)
     */
    std::vector<int> getDecompositionInfo();

    /**
     * @brief Clear all nodes and edges from the graph
     */
    void clear();

    /**
     * @brief Get information about a specific node
     * @param nodeId Node ID to query
     * @return String description of the node
     */
    std::string getNodeInfo(int nodeId) const;

    /**
     * @brief Get all nodes in the graph (for visualization)
     * @return Vector of NodeInfo for all nodes in the graph
     */
    std::vector<NodeInfo> getNodes() const;

    /**
     * @brief Get all edges/constraints in the graph (for visualization)
     * @return Vector of EdgeInfo for all edges in the graph
     */
    std::vector<EdgeInfo> getEdges() const;

    /**
     * @brief Decompose the graph into subgraphs and return full structure
     * @return Vector of SubgraphInfo, each containing nodes and edges for that subgraph
     */
    std::vector<SubgraphInfo> getDecomposedSubgraphs();
};

} // namespace GcsBinding

#endif // CONSTRAINT_GRAPH_HANDLE_HPP
