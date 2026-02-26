#include "constraint_model.hpp"

// General STD/STL headers
#include <format>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

// Custom headers
#include <constraints.hpp>
#include <elements.hpp>
#include <gcs_data_structures.hpp>
#include <geometric_constraint_system.hpp>

// Thirdparty headers
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace Gui {

ConstraintModel::ConstraintModel() = default;

ElementId ConstraintModel::addPoint(double x, double y)
{
    auto nodeId = m_constraintGraph.getGraph().addNode();
    auto element
        = std::make_shared<Gcs::Element>(Gcs::Point(Eigen::Vector2d(x, y)));

    m_constraintGraph.addElement(nodeId, element);

    ElementId eid = m_nextElementId++;
    m_elemToNode[eid] = nodeId;
    m_nodeToElem[nodeId.value] = eid;

    notifyChange();
    return eid;
}

ElementId ConstraintModel::addLine(double x1, double y1, double x2, double y2)
{
    auto nodeId = m_constraintGraph.getGraph().addNode();
    auto element = std::make_shared<Gcs::Element>(
        Gcs::Line(Eigen::Vector2d(x1, y1), Eigen::Vector2d(x2, y2)));

    m_constraintGraph.addElement(nodeId, element);

    ElementId eid = m_nextElementId++;
    m_elemToNode[eid] = nodeId;
    m_nodeToElem[nodeId.value] = eid;

    notifyChange();
    return eid;
}

std::optional<ConstraintId> ConstraintModel::addDistanceConstraint(
    ElementId elemA, ElementId elemB, double distance)
{
    auto itA = m_elemToNode.find(elemA);
    auto itB = m_elemToNode.find(elemB);
    if (itA == m_elemToNode.end() || itB == m_elemToNode.end())
        return std::nullopt;

    // Reject distance constraints between two Line elements —
    // perpendicular distance between two lines is not a
    // meaningful geometric constraint in this solver.
    auto elementA = m_constraintGraph.getElement(itA->second);
    auto elementB = m_constraintGraph.getElement(itB->second);
    if (elementA && elementB && elementA->isElementType<Gcs::Line>()
        && elementB->isElementType<Gcs::Line>()) {
        return std::nullopt;
    }

    auto edgeResult
        = m_constraintGraph.getGraph().addEdge(itA->second, itB->second);
    if (!edgeResult.has_value())
        return std::nullopt;

    auto edgeId = edgeResult.value();
    auto constraint
        = std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(distance));

    m_constraintGraph.addConstraint(edgeId, constraint);

    ConstraintId cid = m_nextConstraintId++;
    m_constrToEdge[cid] = edgeId;
    m_edgeToConstr[edgeId.value] = cid;

    notifyChange();
    return cid;
}

bool ConstraintModel::removeElement(ElementId id)
{
    auto it = m_elemToNode.find(id);
    if (it == m_elemToNode.end())
        return false;

    auto nodeId = it->second;

    // Find all constraints (edges) incident to this node and remove
    // them from our mappings first.
    auto& graph = m_constraintGraph.getGraph();
    if (graph.hasNode(nodeId)) {
        // Copy the edge set since removal modifies it
        auto edges = graph.getEdges(nodeId);
        for (const auto& edgeId : edges) {
            auto constrIt = m_edgeToConstr.find(edgeId.value);
            if (constrIt != m_edgeToConstr.end()) {
                m_constrToEdge.erase(constrIt->second);
                m_edgeToConstr.erase(constrIt);
            }
        }
    }

    m_constraintGraph.removeElement(nodeId);
    m_nodeToElem.erase(nodeId.value);
    m_elemToNode.erase(it);

    notifyChange();
    return true;
}

bool ConstraintModel::removeConstraint(ConstraintId id)
{
    auto it = m_constrToEdge.find(id);
    if (it == m_constrToEdge.end())
        return false;

    auto edgeId = it->second;
    m_constraintGraph.removeConstraintEdge(edgeId);
    m_edgeToConstr.erase(edgeId.value);
    m_constrToEdge.erase(it);

    notifyChange();
    return true;
}

void ConstraintModel::updateElementPosition(ElementId id, double x, double y)
{
    auto it = m_elemToNode.find(id);
    if (it == m_elemToNode.end())
        return;

    auto elem = m_constraintGraph.getElement(it->second);
    if (!elem)
        return;

    if (elem->isElementType<Gcs::Point>()) {
        auto& point = elem->getElement<Gcs::Point>();
        point.canvasPosition = Eigen::Vector2d(x, y);
    }
    notifyChange();
}

void ConstraintModel::updateLinePosition(
    ElementId id, double x1, double y1, double x2, double y2)
{
    auto it = m_elemToNode.find(id);
    if (it == m_elemToNode.end())
        return;

    auto elem = m_constraintGraph.getElement(it->second);
    if (!elem)
        return;

    if (elem->isElementType<Gcs::Line>()) {
        auto& line = elem->getElement<Gcs::Line>();
        line.canvasP1 = Eigen::Vector2d(x1, y1);
        line.canvasP2 = Eigen::Vector2d(x2, y2);
    }
    notifyChange();
}

std::string ConstraintModel::getStatusText() const
{
    auto nodes = m_constraintGraph.nodeCount();
    auto edges = m_constraintGraph.edgeCount();
    return std::format("Nodes: {}  Edges: {}", nodes, edges);
}

const Gcs::ConstraintGraph& ConstraintModel::getConstraintGraph() const
{
    return m_constraintGraph;
}

std::optional<std::pair<ElementId, ElementId>>
ConstraintModel::getConstraintEndpoints(ConstraintId id) const
{
    auto it = m_constrToEdge.find(id);
    if (it == m_constrToEdge.end())
        return std::nullopt;

    auto edgeId = it->second;
    auto& graph = m_constraintGraph.getGraph();
    if (!graph.hasEdge(edgeId))
        return std::nullopt;

    auto [nodeA, nodeB] = graph.getEndpoints(edgeId);

    auto itA = m_nodeToElem.find(nodeA.value);
    auto itB = m_nodeToElem.find(nodeB.value);
    if (itA == m_nodeToElem.end() || itB == m_nodeToElem.end())
        return std::nullopt;

    return std::pair { itA->second, itB->second };
}

std::optional<double> ConstraintModel::getConstraintValue(ConstraintId id) const
{
    auto it = m_constrToEdge.find(id);
    if (it == m_constrToEdge.end())
        return std::nullopt;

    auto constraint = m_constraintGraph.getConstraintForEdge(it->second);
    if (!constraint)
        return std::nullopt;

    auto val = constraint->getConstraintValue();
    if (val.has_value())
        return val.value();
    return std::nullopt;
}

std::vector<ConstraintId> ConstraintModel::getConstraintsForElement(
    ElementId id) const
{
    std::vector<ConstraintId> result;
    auto it = m_elemToNode.find(id);
    if (it == m_elemToNode.end())
        return result;

    auto nodeId = it->second;
    auto& graph = m_constraintGraph.getGraph();
    if (!graph.hasNode(nodeId))
        return result;

    for (const auto& edgeId : graph.getEdges(nodeId)) {
        auto constrIt = m_edgeToConstr.find(edgeId.value);
        if (constrIt != m_edgeToConstr.end()) {
            result.push_back(constrIt->second);
        }
    }
    return result;
}

std::optional<std::pair<double, double>>
ConstraintModel::getPointCanvasPosition(ElementId id) const
{
    auto it = m_elemToNode.find(id);
    if (it == m_elemToNode.end()) {
        return std::nullopt;
    }

    auto element = m_constraintGraph.getElement(it->second);
    if (!element || !element->isElementType<Gcs::Point>()) {
        return std::nullopt;
    }

    const auto& point = element->getElement<Gcs::Point>();
    return std::pair { point.canvasPosition.x(), point.canvasPosition.y() };
}

std::optional<std::pair<std::pair<double, double>, std::pair<double, double>>>
ConstraintModel::getLineCanvasEndpoints(ElementId id) const
{
    auto it = m_elemToNode.find(id);
    if (it == m_elemToNode.end()) {
        return std::nullopt;
    }

    auto element = m_constraintGraph.getElement(it->second);
    if (!element || !element->isElementType<Gcs::Line>()) {
        return std::nullopt;
    }

    const auto& line = element->getElement<Gcs::Line>();
    return std::pair { std::pair { line.canvasP1.x(), line.canvasP1.y() },
        std::pair { line.canvasP2.x(), line.canvasP2.y() } };
}

bool ConstraintModel::isElementSolved(ElementId id) const
{
    auto it = m_elemToNode.find(id);
    if (it == m_elemToNode.end()) {
        return false;
    }

    auto element = m_constraintGraph.getElement(it->second);
    return element && element->isElementSet();
}

std::string ConstraintModel::solveConstraintSystem()
{
    try {
        Gcs::GeometricConstraintSystem solver(
            std::make_unique<Gcs::DeficitStreeBasedTopDownStrategy>());
        solver.solveGeometricConstraintSystem(m_constraintGraph);
    } catch (const std::runtime_error& error) {
        return error.what();
    }

    applySolverToCanvasTransform();
    notifyChange();
    return "";
}

void ConstraintModel::applySolverToCanvasTransform()
{
    // Collect corresponding point pairs: (solver position, canvas position)
    // for all solved Point elements.
    struct PointPair {
        Eigen::Vector2d solverPosition;
        Eigen::Vector2d canvasPosition;
    };

    std::vector<PointPair> solvedPointPairs;

    const auto& elementMap = m_constraintGraph.getElementMap();
    for (const auto& [nodeId, element] : elementMap) {
        if (!element || !element->isElementSet()) {
            continue;
        }
        if (element->isElementType<Gcs::Point>()) {
            const auto& point = element->getElement<Gcs::Point>();
            solvedPointPairs.push_back(
                { point.position, point.canvasPosition });
        }
    }

    // Need at least 2 points for a meaningful rigid body transform.
    // With 0 points there is nothing to transform; with 1 point we
    // can only translate (rotation is undetermined).
    if (solvedPointPairs.size() < 2) {
        if (solvedPointPairs.size() == 1) {
            // Translation-only fallback: shift so the single solved
            // point lands on its original canvas position.
            Eigen::Vector2d translationOffset
                = solvedPointPairs[0].canvasPosition
                - solvedPointPairs[0].solverPosition;

            for (const auto& [nodeId, element] : elementMap) {
                if (!element || !element->isElementSet()) {
                    continue;
                }
                if (element->isElementType<Gcs::Point>()) {
                    auto& point = element->getElement<Gcs::Point>();
                    point.canvasPosition = point.position + translationOffset;
                } else if (element->isElementType<Gcs::Line>()) {
                    auto& line = element->getElement<Gcs::Line>();
                    line.canvasP1 = line.p1 + translationOffset;
                    line.canvasP2 = line.p2 + translationOffset;
                }
            }
        }
        return;
    }

    // Compute centroids of solver and canvas point sets.
    Eigen::Vector2d solverCentroid = Eigen::Vector2d::Zero();
    Eigen::Vector2d canvasCentroid = Eigen::Vector2d::Zero();

    for (const auto& pair : solvedPointPairs) {
        solverCentroid += pair.solverPosition;
        canvasCentroid += pair.canvasPosition;
    }

    auto pointCount = static_cast<double>(solvedPointPairs.size());
    solverCentroid /= pointCount;
    canvasCentroid /= pointCount;

    // Build the 2x2 cross-covariance matrix H.
    // H = sum( (solver_i - solverCentroid) * (canvas_i - canvasCentroid)^T )
    Eigen::Matrix2d crossCovariance = Eigen::Matrix2d::Zero();

    for (const auto& pair : solvedPointPairs) {
        Eigen::Vector2d solverCentered = pair.solverPosition - solverCentroid;
        Eigen::Vector2d canvasCentered = pair.canvasPosition - canvasCentroid;
        crossCovariance += solverCentered * canvasCentered.transpose();
    }

    // SVD: H = U * S * V^T
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        crossCovariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d matrixU = svd.matrixU();
    Eigen::Matrix2d matrixV = svd.matrixV();

    // Rotation R = V * diag(1, det(V * U^T)) * U^T
    // The determinant correction ensures a proper rotation (not a
    // reflection).
    Eigen::Matrix2d vuTranspose = matrixV * matrixU.transpose();
    double determinant = vuTranspose.determinant();
    Eigen::Matrix2d correctionDiag = Eigen::Matrix2d::Identity();
    correctionDiag(1, 1) = determinant;

    Eigen::Matrix2d rotation = matrixV * correctionDiag * matrixU.transpose();

    // Translation: t = canvasCentroid - R * solverCentroid
    Eigen::Vector2d translation = canvasCentroid - rotation * solverCentroid;

    // Apply the rigid body transform to all solved elements.
    for (const auto& [nodeId, element] : elementMap) {
        if (!element || !element->isElementSet()) {
            continue;
        }
        if (element->isElementType<Gcs::Point>()) {
            auto& point = element->getElement<Gcs::Point>();
            point.canvasPosition = rotation * point.position + translation;
        } else if (element->isElementType<Gcs::Line>()) {
            auto& line = element->getElement<Gcs::Line>();
            line.canvasP1 = rotation * line.p1 + translation;
            line.canvasP2 = rotation * line.p2 + translation;
        }
    }
}

void ConstraintModel::setChangeCallback(ChangeCallback cb)
{
    m_changeCallback = std::move(cb);
}

void ConstraintModel::notifyChange()
{
    if (m_changeCallback)
        m_changeCallback();
}

} // namespace Gui
