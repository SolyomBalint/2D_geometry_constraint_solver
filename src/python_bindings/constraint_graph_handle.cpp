#include "constraint_graph_handle.hpp"
#include <stdexcept>
#include <sstream>

namespace GcsBinding {

namespace {
    // Helper function to extract element info (type and data)
    void extractElementInfo(const std::shared_ptr<Gcs::Element>& element,
        std::string& type, std::vector<double>& data, bool flipY)
    {
        element->visitElement([&type, &data, flipY](const auto& e) {
            using T = std::decay_t<decltype(e)>;

            if constexpr (std::is_same_v<T, Gcs::Point>) {
                type = "Point";
                double y = flipY ? -e.y : e.y;
                data = { e.x, y };
            } else if constexpr (std::is_same_v<T, Gcs::FixedRadiusCircle>) {
                type = "Circle";
                double y = flipY ? -e.y : e.y;
                data = { e.x, y, e.fixed_radius };
            } else if constexpr (std::is_same_v<T, Gcs::Line>) {
                type = "Line";
                double y1 = flipY ? -e.y1 : e.y1;
                double y2 = flipY ? -e.y2 : e.y2;
                data = { e.x1, y1, e.x2, y2 };
            }
        });
    }

    // Helper function to extract constraint info (type and value)
    void extractConstraintInfo(
        const std::shared_ptr<Gcs::Constraint>& constraint, std::string& type,
        double& value)
    {
        constraint->visitConsraint([&type, &value](const auto& c) {
            using T = std::decay_t<decltype(c)>;

            if constexpr (std::is_same_v<T, Gcs::DistanceConstraint>) {
                type = "Distance";
                value = c.distance;
            } else if constexpr (std::is_same_v<T, Gcs::TangencyConstraint>) {
                type = "Tangency";
                value = c.angle;
            } else if constexpr (std::is_same_v<T, Gcs::PointOnLineConstraint>) {
                type = "PointOnLine";
                value = 0.0; // No value for point-on-line constraint
            }
        });
    }
} // anonymous namespace

int ConstraintGraphHandle::addPoint(double canvasX, double canvasY)
{
    // Create Point with canvas coordinates stored, flip Y for solver space
    auto point = std::make_shared<Gcs::Element>(
        Gcs::Point(canvasX, flipYForSolver(canvasY)));

    auto& node = graph_.addNode(point);
    int nodeId = nextNodeId_++;
    nodeIdMap_[nodeId] = &node;

    return nodeId;
}

int ConstraintGraphHandle::addCircle(
    double canvasX, double canvasY, double radius)
{
    // Create FixedRadiusCircle, flip Y for solver space
    auto circle = std::make_shared<Gcs::Element>(
        Gcs::FixedRadiusCircle(canvasX, flipYForSolver(canvasY), radius));

    auto& node = graph_.addNode(circle);
    int nodeId = nextNodeId_++;
    nodeIdMap_[nodeId] = &node;

    return nodeId;
}

int ConstraintGraphHandle::addLine(double x1, double y1, double x2, double y2)
{
    // Create Line with two points, flip Y coordinates for solver space
    auto line = std::make_shared<Gcs::Element>(
        Gcs::Line(x1, flipYForSolver(y1), x2, flipYForSolver(y2)));

    auto& node = graph_.addNode(line);
    int nodeId = nextNodeId_++;
    nodeIdMap_[nodeId] = &node;

    return nodeId;
}

bool ConstraintGraphHandle::addDistanceConstraint(
    int nodeId1, int nodeId2, double distance)
{
    auto it1 = nodeIdMap_.find(nodeId1);
    auto it2 = nodeIdMap_.find(nodeId2);

    if (it1 == nodeIdMap_.end() || it2 == nodeIdMap_.end()) {
        return false;
    }

    auto constraint
        = std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(distance));

    graph_.addEdge(*it1->second, *it2->second, constraint);
    return true;
}

bool ConstraintGraphHandle::addTangencyConstraint(
    int nodeId1, int nodeId2, double angle)
{
    auto it1 = nodeIdMap_.find(nodeId1);
    auto it2 = nodeIdMap_.find(nodeId2);

    if (it1 == nodeIdMap_.end() || it2 == nodeIdMap_.end()) {
        return false;
    }

    auto constraint
        = std::make_shared<Gcs::Constraint>(Gcs::TangencyConstraint(angle));

    graph_.addEdge(*it1->second, *it2->second, constraint);
    return true;
}

bool ConstraintGraphHandle::addPointOnLineConstraint(
    int pointNodeId, int lineNodeId)
{
    auto itPoint = nodeIdMap_.find(pointNodeId);
    auto itLine = nodeIdMap_.find(lineNodeId);

    if (itPoint == nodeIdMap_.end() || itLine == nodeIdMap_.end()) {
        return false;
    }

    auto constraint
        = std::make_shared<Gcs::Constraint>(Gcs::PointOnLineConstraint());

    graph_.addEdge(*itPoint->second, *itLine->second, constraint);
    return true;
}

std::vector<NodePosition> ConstraintGraphHandle::solve()
{
    // Run the full GCS pipeline
    Gcs::GeometricConstraintSystem solver;
    solver.solveGcsViaPipeline(graph_);

    // Extract solved positions and convert back to canvas space
    std::vector<NodePosition> positions;

    for (const auto& [nodeId, nodePtr] : nodeIdMap_) {
        auto element = nodePtr->getStoredObj();
        std::string type;
        std::vector<double> data;
        extractElementInfo(
            element, type, data, true); // true = flip Y for canvas
        positions.emplace_back(nodeId, data);
    }

    return positions;
}

bool ConstraintGraphHandle::isWellConstrained() const
{
    return Gcs::defaultDetectorFunc(graph_);
}

int ConstraintGraphHandle::getNodeCount() const
{
    return static_cast<int>(graph_.getNodeCount());
}

int ConstraintGraphHandle::getEdgeCount() const
{
    return static_cast<int>(graph_.getEdgeCount());
}

std::vector<int> ConstraintGraphHandle::getDecompositionInfo()
{
    try {
        auto subgraphs = Gcs::defaultDecompositorFunc(graph_);

        std::vector<int> sizes;
        sizes.reserve(subgraphs.size());

        for (const auto& subgraph : subgraphs) {
            sizes.push_back(static_cast<int>(subgraph.getNodeCount()));
        }

        return sizes;
    } catch (const std::exception& e) {
        // If decomposition fails (e.g., graph not biconnected), return empty
        // vector
        return {};
    }
}

void ConstraintGraphHandle::clear()
{
    // Create a new empty graph and reset node mappings
    graph_ = Gcs::ConstraintGraph();
    nodeIdMap_.clear();
    nextNodeId_ = 0;
}

std::string ConstraintGraphHandle::getNodeInfo(int nodeId) const
{
    auto it = nodeIdMap_.find(nodeId);
    if (it == nodeIdMap_.end()) {
        return "Node not found";
    }

    auto element = it->second->getStoredObj();
    std::ostringstream oss;

    oss << "Node " << nodeId << ": ";
    oss << element->toString();

    return oss.str();
}

std::vector<NodeInfo> ConstraintGraphHandle::getNodes() const
{
    std::vector<NodeInfo> nodes;
    nodes.reserve(nodeIdMap_.size());

    for (const auto& [nodeId, nodePtr] : nodeIdMap_) {
        auto element = nodePtr->getStoredObj();
        std::string type;
        std::vector<double> data;
        extractElementInfo(
            element, type, data, true); // true = flip Y for canvas
        nodes.emplace_back(nodeId, type, data);
    }

    return nodes;
}

std::vector<EdgeInfo> ConstraintGraphHandle::getEdges() const
{
    std::vector<EdgeInfo> edges;

    // Need to check all pairs since Edge interface doesn't expose connected
    // nodes
    std::vector<std::pair<int, Gcs::ConstraintGraph::NodeType*>> nodeList(
        nodeIdMap_.begin(), nodeIdMap_.end());

    for (size_t i = 0; i < nodeList.size(); ++i) {
        for (size_t j = i + 1; j < nodeList.size(); ++j) {
            int nodeId1 = nodeList[i].first;
            int nodeId2 = nodeList[j].first;
            auto* node1Ptr = nodeList[i].second;
            auto* node2Ptr = nodeList[j].second;

            auto edgeOpt = graph_.getEdgeBetweenNodes(*node1Ptr, *node2Ptr);
            if (edgeOpt.has_value()) {
                auto constraint = edgeOpt->getStoredObj();
                std::string type;
                double value = 0.0;
                extractConstraintInfo(constraint, type, value);
                bool isVirtual = edgeOpt->isVirtual();
                edges.emplace_back(nodeId1, nodeId2, type, value, isVirtual);
            }
        }
    }

    return edges;
}

std::vector<SubgraphInfo> ConstraintGraphHandle::getDecomposedSubgraphs()
{
    try {
        // Call the C++ decomposition function
        auto subgraphs = Gcs::defaultDecompositorFunc(graph_);

        std::vector<SubgraphInfo> result;
        result.reserve(subgraphs.size());

        // Process each subgraph
        for (const auto& subgraph : subgraphs) {
            std::vector<NodeInfo> subgraphNodes;
            std::vector<EdgeInfo> subgraphEdges;

            // Map from subgraph node pointers to sequential IDs for this
            // subgraph
            std::unordered_map<const Gcs::ConstraintGraph::NodeType*, int>
                localNodeIdMap;
            int localNodeId = 0;

            // Get all nodes from this subgraph
            auto nodes = subgraph.getNodes();
            subgraphNodes.reserve(nodes.size());

            for (const auto& node : nodes) {
                auto element = node.getStoredObj();
                std::string type;
                std::vector<double> data;
                extractElementInfo(
                    element, type, data, true); // true = flip Y for canvas
                subgraphNodes.emplace_back(localNodeId, type, data);
                localNodeIdMap[&node] = localNodeId;
                localNodeId++;
            }

            // Get all edges from this subgraph by checking all node pairs
            for (size_t i = 0; i < nodes.size(); ++i) {
                for (size_t j = i + 1; j < nodes.size(); ++j) {
                    auto edgeOpt
                        = subgraph.getEdgeBetweenNodes(nodes[i], nodes[j]);
                    if (edgeOpt.has_value()) {
                        // Map nodes to local IDs
                        auto it1 = localNodeIdMap.find(&nodes[i]);
                        auto it2 = localNodeIdMap.find(&nodes[j]);

                        if (it1 != localNodeIdMap.end()
                            && it2 != localNodeIdMap.end()) {
                            int nodeId1 = it1->second;
                            int nodeId2 = it2->second;

                            auto constraint = edgeOpt->getStoredObj();
                            std::string type;
                            double value = 0.0;
                            extractConstraintInfo(constraint, type, value);
                            bool isVirtual = edgeOpt->isVirtual();
                            subgraphEdges.emplace_back(
                                nodeId1, nodeId2, type, value, isVirtual);
                        }
                    }
                }
            }

            result.emplace_back(subgraphNodes, subgraphEdges);
        }

        return result;

    } catch (const std::exception& e) {
        // If decomposition fails (e.g., graph not biconnected), return empty
        // vector
        return {};
    }
}

} // namespace GcsBinding
