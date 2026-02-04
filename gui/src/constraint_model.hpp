#ifndef GUI_CONSTRAINT_MODEL_HPP
#define GUI_CONSTRAINT_MODEL_HPP

// General STD/STL headers
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Custom headers
#include "./canvas_types.hpp"

// Constraint solver headers
#include <gcs_data_structures.hpp>

namespace Gui {

/**
 * @brief Bridge between the GUI canvas and the constraint solver's
 *        ConstraintGraph.
 *
 * Owns the ConstraintGraph and maintains bidirectional mappings between
 * canvas element IDs and graph node/edge IDs.
 */
class ConstraintModel {
public:
    using ChangeCallback = std::function<void()>;

    ConstraintModel();

    /**
     * @brief Add a point to the constraint graph.
     * @param canvasPos The position on the canvas.
     * @return The canvas element ID assigned to the new point.
     */
    ElementId addPoint(double x, double y);

    /**
     * @brief Add a line to the constraint graph.
     * @param x1 First endpoint x.
     * @param y1 First endpoint y.
     * @param x2 Second endpoint x.
     * @param y2 Second endpoint y.
     * @return The canvas element ID assigned to the new line.
     */
    ElementId addLine(double x1, double y1, double x2, double y2);

    /**
     * @brief Add a distance constraint between two elements.
     * @param elemA First element ID.
     * @param elemB Second element ID.
     * @param distance The distance value.
     * @return The constraint ID if successful, std::nullopt otherwise.
     */
    std::optional<ConstraintId> addDistanceConstraint(
        ElementId elemA, ElementId elemB, double distance);

    /**
     * @brief Remove an element and all its associated constraints.
     * @param id The canvas element ID to remove.
     * @return true if the element was found and removed.
     */
    bool removeElement(ElementId id);

    /**
     * @brief Remove a constraint.
     * @param id The constraint ID to remove.
     * @return true if the constraint was found and removed.
     */
    bool removeConstraint(ConstraintId id);

    /**
     * @brief Update an element's canvas position (for dragging).
     * @param id The element ID.
     * @param x New x coordinate.
     * @param y New y coordinate.
     */
    void updateElementPosition(ElementId id, double x, double y);

    /**
     * @brief Update a line element's endpoint positions.
     * @param id The line element ID.
     * @param x1 New first endpoint x.
     * @param y1 New first endpoint y.
     * @param x2 New second endpoint x.
     * @param y2 New second endpoint y.
     */
    void updateLinePosition(
        ElementId id, double x1, double y1, double x2, double y2);

    /**
     * @brief Get info about the constraint graph for display.
     * @return A string describing the current state.
     */
    std::string getStatusText() const;

    /**
     * @brief Get the constraint graph (read-only).
     * @return Const reference to the underlying ConstraintGraph.
     */
    const Gcs::ConstraintGraph& getConstraintGraph() const;

    /**
     * @brief Get constraint info for a given constraint ID.
     * @param id The constraint ID.
     * @return Pair of element IDs that the constraint connects.
     */
    std::optional<std::pair<ElementId, ElementId>> getConstraintEndpoints(
        ConstraintId id) const;

    /**
     * @brief Get the distance value for a constraint.
     * @param id The constraint ID.
     * @return The distance value, or std::nullopt if not a distance
     *         constraint.
     */
    std::optional<double> getConstraintValue(ConstraintId id) const;

    /**
     * @brief Get all constraint IDs associated with an element.
     * @param id The element ID.
     * @return Vector of constraint IDs.
     */
    std::vector<ConstraintId> getConstraintsForElement(ElementId id) const;

    /**
     * @brief Get the canvas position of a point element.
     * @param id The element ID.
     * @return The (x, y) canvas position, or std::nullopt if the
     *         element is not a point or doesn't exist.
     */
    std::optional<std::pair<double, double>> getPointCanvasPosition(
        ElementId id) const;

    /**
     * @brief Get the canvas endpoint positions of a line element.
     * @param id The element ID.
     * @return The ((x1,y1), (x2,y2)) canvas endpoint positions, or
     *         std::nullopt if the element is not a line or doesn't
     *         exist.
     */
    std::optional<
        std::pair<std::pair<double, double>, std::pair<double, double>>>
    getLineCanvasEndpoints(ElementId id) const;

    /**
     * @brief Check whether an element has been solved.
     * @param id The element ID.
     * @return true if the element exists and has been solved.
     */
    bool isElementSolved(ElementId id) const;

    /**
     * @brief Run the constraint solver on the current graph.
     *
     * Creates a GeometricConstraintSystem with
     * DeficitStreeBasedTopDownStrategy, solves the constraint graph,
     * then applies a rigid body (Procrustes) transform to map
     * solver-computed positions back into canvas world coordinates.
     *
     * @return Empty string on success, or an error message describing
     *         why solving failed.
     */
    std::string solveConstraintSystem();

    /**
     * @brief Register a callback for when the model changes.
     * @param cb The callback function.
     */
    void setChangeCallback(ChangeCallback cb);

private:
    /**
     * @brief Apply a rigid body transform to map solver positions
     *        back to canvas world coordinates.
     *
     * Uses Procrustes analysis (SVD) without scaling to find the
     * optimal rotation and translation that maps solver-computed
     * positions onto original canvas positions. Only elements marked
     * as solved (@c isElementSet()) are transformed.
     */
    void applySolverToCanvasTransform();
    void notifyChange();

    Gcs::ConstraintGraph m_constraintGraph;

    // Bidirectional mappings
    std::unordered_map<ElementId, MathUtils::NodeId> m_elemToNode;
    std::unordered_map<int, ElementId>
        m_nodeToElem; // NodeId.value -> ElementId

    std::unordered_map<ConstraintId, MathUtils::EdgeId> m_constrToEdge;
    std::unordered_map<int, ConstraintId>
        m_edgeToConstr; // EdgeId.value -> ConstraintId

    ElementId m_nextElementId { 0 };
    ConstraintId m_nextConstraintId { 0 };

    ChangeCallback m_changeCallback;
};

} // namespace Gui

#endif // GUI_CONSTRAINT_MODEL_HPP
