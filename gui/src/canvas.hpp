#ifndef GUI_CANVAS_HPP
#define GUI_CANVAS_HPP

// General STD/STL headers
#include <optional>
#include <unordered_map>
#include <vector>

// Custom headers
#include "./canvas_types.hpp"
#include "./constraint_model.hpp"

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief Custom drawing area that renders geometric elements
 *        and handles user interaction for the modeller.
 *
 * Manages a list of visual elements (points, lines) and constraints,
 * synchronized with the ConstraintModel. Supports pan, zoom, and
 * tool-based interaction.
 */
class Canvas : public Gtk::DrawingArea {
public:
    explicit Canvas(ConstraintModel& model);

    /** @brief Set the currently active tool. */
    void setTool(Tool tool);

    /** @brief Get the currently active tool. */
    Tool getTool() const;

    /** @brief Clear selection on all elements. */
    void clearSelection();

    /** @brief Get the currently selected element, if any. */
    std::optional<ElementId> getSelectedElement() const;

    /** @brief Get all canvas elements. */
    const std::unordered_map<ElementId, CanvasElement>& getElements() const;

    /** @brief Get all canvas constraints. */
    const std::unordered_map<ConstraintId, CanvasConstraint>&
    getConstraints() const;

    /**
     * @brief Refresh visual element positions from the model.
     *
     * Reads the current canvas positions from the ConstraintModel's
     * underlying Element objects and updates the corresponding
     * CanvasElement records. Only solved elements are updated.
     * Triggers a redraw.
     */
    void refreshPositionsFromModel();

    /**
     * @brief Signal emitted when status text changes.
     * @return The signal accessor.
     */
    using StatusSignal = sigc::signal<void(const Glib::ustring&)>;
    StatusSignal signalStatusChanged();

    /**
     * @brief Signal emitted when a distance constraint dialog should
     *        be shown.
     *
     * The two element IDs are the elements to constrain.
     */
    using ConstraintRequestSignal = sigc::signal<void(ElementId, ElementId)>;
    ConstraintRequestSignal signalConstraintRequested();

    /**
     * @brief Signal emitted when an angle constraint dialog should
     *        be shown.
     *
     * The two element IDs are the line elements to constrain.
     */
    using AngleConstraintRequestSignal
        = sigc::signal<void(ElementId, ElementId)>;
    AngleConstraintRequestSignal signalAngleConstraintRequested();

    /**
     * @brief Add a constraint record to the canvas's visual state.
     * @param constraint The canvas constraint to add.
     */
    void addCanvasConstraint(const CanvasConstraint& constraint);

private:
    // Drawing
    void onDraw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);
    void drawGrid(
        const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);
    void drawElements(const Cairo::RefPtr<Cairo::Context>& cr);
    void drawConstraints(const Cairo::RefPtr<Cairo::Context>& cr);
    void drawDistanceConstraint(const Cairo::RefPtr<Cairo::Context>& cr,
        const CanvasConstraint& constraint, const CanvasElement& elementA,
        const CanvasElement& elementB, double fontSize);
    void drawAngleConstraint(const Cairo::RefPtr<Cairo::Context>& cr,
        const CanvasConstraint& constraint, const CanvasElement& lineA,
        const CanvasElement& lineB, double fontSize);
    void drawPendingLine(const Cairo::RefPtr<Cairo::Context>& cr);

    // Hit testing
    std::optional<ElementId> hitTest(double x, double y) const;
    std::optional<ConstraintId> hitTestConstraint(double x, double y) const;

    // Coordinate transforms
    void screenToWorld(double sx, double sy, double& wx, double& wy) const;
    void worldToScreen(double wx, double wy, double& sx, double& sy) const;

    // Event handlers
    void onClickPressed(int nPress, double x, double y);
    void onClickReleased(int nPress, double x, double y);
    void onDragBegin(double startX, double startY);
    void onDragUpdate(double offsetX, double offsetY);
    void onDragEnd(double offsetX, double offsetY);
    void onMotion(double x, double y);
    bool onScroll(double dx, double dy);
    bool onKeyPressed(guint keyval, guint keycode, Gdk::ModifierType state);

    // Tool handlers
    void handleSelectClick(double wx, double wy);
    void handlePointClick(double wx, double wy);
    void handleLineClick(double wx, double wy);
    void handleConstraintClick(double wx, double wy);
    void handleAngleConstraintClick(double wx, double wy);
    void handleDeleteClick(double wx, double wy);

    void updateStatus();

    ConstraintModel& m_model;
    Tool m_currentTool = Tool::Select;

    // Visual elements
    std::unordered_map<ElementId, CanvasElement> m_elements;
    std::unordered_map<ConstraintId, CanvasConstraint> m_constraints;

    // Selection state
    std::optional<ElementId> m_selectedElement;
    std::optional<ElementId> m_constraintFirstElement;
    std::optional<ElementId> m_angleConstraintFirstElement;

    // Line creation state
    bool m_lineFirstPointSet = false;
    double m_lineFirstX = 0.0;
    double m_lineFirstY = 0.0;

    // Dragging state
    bool m_isDragging = false;
    bool m_isPanning = false;
    double m_dragStartWx = 0.0;
    double m_dragStartWy = 0.0;
    double m_dragOrigX = 0.0;
    double m_dragOrigY = 0.0;
    double m_dragOrigX2 = 0.0;
    double m_dragOrigY2 = 0.0;

    // Pan and zoom
    double m_panX = 0.0;
    double m_panY = 0.0;
    double m_zoom = 1.0;

    // Mouse position for pending line preview
    double m_mouseWorldX = 0.0;
    double m_mouseWorldY = 0.0;

    // Signals
    StatusSignal m_statusSignal;
    ConstraintRequestSignal m_constraintRequestSignal;
    AngleConstraintRequestSignal m_angleConstraintRequestSignal;

    // Gesture controllers (prevent premature destruction)
    Glib::RefPtr<Gtk::GestureClick> m_clickGesture;
    Glib::RefPtr<Gtk::GestureDrag> m_dragGesture;
    Glib::RefPtr<Gtk::EventControllerMotion> m_motionController;
    Glib::RefPtr<Gtk::EventControllerScroll> m_scrollController;
    Glib::RefPtr<Gtk::EventControllerKey> m_keyController;
};

} // namespace Gui

#endif // GUI_CANVAS_HPP
