#include "canvas.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

namespace Gui {

namespace {
    constexpr double POINT_RADIUS = 6.0;
    constexpr double HIT_TOLERANCE = 10.0;
    constexpr double GRID_SPACING = 50.0;
    constexpr double ZOOM_FACTOR = 1.1;
    constexpr double MIN_ZOOM = 0.1;
    constexpr double MAX_ZOOM = 20.0;

    constexpr double CONSTRAINT_LABEL_OFFSET = 12.0;
} // namespace

Canvas::Canvas(ConstraintModel& model)
    : m_model(model)
{
    set_focusable(true);
    set_can_focus(true);
    set_hexpand(true);
    set_vexpand(true);

    set_draw_func(sigc::mem_fun(*this, &Canvas::onDraw));

    // Click gesture for tool actions
    m_clickGesture = Gtk::GestureClick::create();
    m_clickGesture->set_button(GDK_BUTTON_PRIMARY);
    m_clickGesture->signal_pressed().connect(
        sigc::mem_fun(*this, &Canvas::onClickPressed));
    m_clickGesture->signal_released().connect(
        sigc::mem_fun(*this, &Canvas::onClickReleased));
    add_controller(m_clickGesture);

    // Drag gesture for moving elements and panning
    m_dragGesture = Gtk::GestureDrag::create();
    m_dragGesture->set_button(GDK_BUTTON_PRIMARY);
    m_dragGesture->signal_drag_begin().connect(
        sigc::mem_fun(*this, &Canvas::onDragBegin));
    m_dragGesture->signal_drag_update().connect(
        sigc::mem_fun(*this, &Canvas::onDragUpdate));
    m_dragGesture->signal_drag_end().connect(
        sigc::mem_fun(*this, &Canvas::onDragEnd));
    add_controller(m_dragGesture);

    // Motion controller for mouse tracking
    m_motionController = Gtk::EventControllerMotion::create();
    m_motionController->signal_motion().connect(
        sigc::mem_fun(*this, &Canvas::onMotion));
    add_controller(m_motionController);

    // Scroll controller for zoom
    m_scrollController = Gtk::EventControllerScroll::create();
    m_scrollController->set_flags(Gtk::EventControllerScroll::Flags::VERTICAL);
    m_scrollController->signal_scroll().connect(
        sigc::mem_fun(*this, &Canvas::onScroll), false);
    add_controller(m_scrollController);

    // Key controller for keyboard shortcuts
    m_keyController = Gtk::EventControllerKey::create();
    m_keyController->signal_key_pressed().connect(
        sigc::mem_fun(*this, &Canvas::onKeyPressed), false);
    add_controller(m_keyController);

    // Listen for model changes
    m_model.setChangeCallback([this]() { queue_draw(); });
}

void Canvas::setTool(Tool tool)
{
    m_currentTool = tool;
    m_lineFirstPointSet = false;
    m_constraintFirstElement.reset();
    updateStatus();
    queue_draw();
}

Tool Canvas::getTool() const
{
    return m_currentTool;
}

void Canvas::clearSelection()
{
    for (auto& [id, elem] : m_elements) {
        elem.selected = false;
    }
    m_selectedElement.reset();
    queue_draw();
}

std::optional<ElementId> Canvas::getSelectedElement() const
{
    return m_selectedElement;
}

const std::unordered_map<ElementId, CanvasElement>& Canvas::getElements() const
{
    return m_elements;
}

const std::unordered_map<ConstraintId, CanvasConstraint>&
Canvas::getConstraints() const
{
    return m_constraints;
}

void Canvas::refreshPositionsFromModel()
{
    for (auto& [elementId, canvasElement] : m_elements) {
        if (!m_model.isElementSolved(elementId)) {
            continue;
        }

        if (canvasElement.type == CanvasElementType::Point) {
            auto position = m_model.getPointCanvasPosition(elementId);
            if (position.has_value()) {
                canvasElement.x = position->first;
                canvasElement.y = position->second;
            }
        } else if (canvasElement.type == CanvasElementType::Line) {
            auto endpoints = m_model.getLineCanvasEndpoints(elementId);
            if (endpoints.has_value()) {
                canvasElement.x = endpoints->first.first;
                canvasElement.y = endpoints->first.second;
                canvasElement.x2 = endpoints->second.first;
                canvasElement.y2 = endpoints->second.second;
            }
        }
    }
    queue_draw();
}

Canvas::StatusSignal Canvas::signalStatusChanged()
{
    return m_statusSignal;
}

Canvas::ConstraintRequestSignal Canvas::signalConstraintRequested()
{
    return m_constraintRequestSignal;
}

// ============================================================
// Drawing
// ============================================================

void Canvas::onDraw(
    const Cairo::RefPtr<Cairo::Context>& cr, int width, int height)
{
    // White background
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->paint();

    // Apply pan/zoom transform
    cr->save();
    cr->translate(m_panX, m_panY);
    cr->scale(m_zoom, m_zoom);

    drawGrid(cr, width, height);
    drawConstraints(cr);
    drawElements(cr);
    drawPendingLine(cr);

    cr->restore();
}

void Canvas::drawGrid(
    const Cairo::RefPtr<Cairo::Context>& cr, int width, int height)
{
    // Compute visible world bounds
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    screenToWorld(0, 0, x0, y0);
    screenToWorld(
        static_cast<double>(width), static_cast<double>(height), x1, y1);

    double gridStep = GRID_SPACING;

    // Adjust grid step for zoom level
    while (gridStep * m_zoom < 20.0)
        gridStep *= 2.0;
    while (gridStep * m_zoom > 100.0)
        gridStep /= 2.0;

    double startX = std::floor(x0 / gridStep) * gridStep;
    double startY = std::floor(y0 / gridStep) * gridStep;

    cr->set_line_width(0.5 / m_zoom);
    cr->set_source_rgba(0.85, 0.85, 0.85, 1.0);

    for (double gx = startX; gx <= x1; gx += gridStep) {
        cr->move_to(gx, y0);
        cr->line_to(gx, y1);
    }
    for (double gy = startY; gy <= y1; gy += gridStep) {
        cr->move_to(x0, gy);
        cr->line_to(x1, gy);
    }
    cr->stroke();

    // Draw axes with slightly darker color
    cr->set_source_rgba(0.6, 0.6, 0.6, 1.0);
    cr->set_line_width(1.0 / m_zoom);

    // X axis
    cr->move_to(x0, 0);
    cr->line_to(x1, 0);
    cr->stroke();

    // Y axis
    cr->move_to(0, y0);
    cr->line_to(0, y1);
    cr->stroke();
}

void Canvas::drawElements(const Cairo::RefPtr<Cairo::Context>& cr)
{
    double pointRadius = POINT_RADIUS / m_zoom;
    double lineWidth = 2.0 / m_zoom;

    for (const auto& [id, elem] : m_elements) {
        if (elem.type == CanvasElementType::Point) {
            // Draw point
            if (elem.selected) {
                cr->set_source_rgb(0.0, 0.5, 1.0);
            } else {
                cr->set_source_rgb(0.2, 0.2, 0.2);
            }
            cr->arc(elem.x, elem.y, pointRadius, 0, 2.0 * std::numbers::pi);
            cr->fill();

            // Draw outline
            cr->set_source_rgb(0.0, 0.0, 0.0);
            cr->set_line_width(1.0 / m_zoom);
            cr->arc(elem.x, elem.y, pointRadius, 0, 2.0 * std::numbers::pi);
            cr->stroke();
        } else if (elem.type == CanvasElementType::Line) {
            // Draw line
            if (elem.selected) {
                cr->set_source_rgb(0.0, 0.5, 1.0);
            } else {
                cr->set_source_rgb(0.1, 0.1, 0.1);
            }
            cr->set_line_width(lineWidth);
            cr->move_to(elem.x, elem.y);
            cr->line_to(elem.x2, elem.y2);
            cr->stroke();

            // Draw endpoint dots
            double epRadius = pointRadius * 0.6;
            cr->set_source_rgb(0.3, 0.3, 0.3);
            cr->arc(elem.x, elem.y, epRadius, 0, 2.0 * std::numbers::pi);
            cr->fill();
            cr->arc(elem.x2, elem.y2, epRadius, 0, 2.0 * std::numbers::pi);
            cr->fill();
        }
    }
}

void Canvas::drawConstraints(const Cairo::RefPtr<Cairo::Context>& cr)
{
    double lineWidth = 1.5 / m_zoom;
    double fontSize = 11.0 / m_zoom;

    for (const auto& [cid, constr] : m_constraints) {
        auto itA = m_elements.find(constr.elementA);
        auto itB = m_elements.find(constr.elementB);
        if (itA == m_elements.end() || itB == m_elements.end())
            continue;

        const auto& elemA = itA->second;
        const auto& elemB = itB->second;

        // Compute center points for each element
        double ax = elemA.x;
        double ay = elemA.y;
        double bx = elemB.x;
        double by = elemB.y;

        if (elemA.type == CanvasElementType::Line) {
            ax = (elemA.x + elemA.x2) / 2.0;
            ay = (elemA.y + elemA.y2) / 2.0;
        }
        if (elemB.type == CanvasElementType::Line) {
            bx = (elemB.x + elemB.x2) / 2.0;
            by = (elemB.y + elemB.y2) / 2.0;
        }

        // Draw dashed line between elements
        cr->set_source_rgba(0.8, 0.2, 0.2, 0.7);
        cr->set_line_width(lineWidth);
        std::vector<double> dashes = { 6.0 / m_zoom, 4.0 / m_zoom };
        cr->set_dash(dashes, 0);
        cr->move_to(ax, ay);
        cr->line_to(bx, by);
        cr->stroke();
        cr->unset_dash();

        // Draw distance label at midpoint
        double mx = (ax + bx) / 2.0;
        double my = (ay + by) / 2.0;

        cr->set_source_rgb(0.8, 0.1, 0.1);
        cr->set_font_size(fontSize);

        std::string label = std::format("{:.1f}", constr.value);
        Cairo::TextExtents extents;
        cr->get_text_extents(label, extents);

        cr->move_to(
            mx - extents.width / 2.0, my - CONSTRAINT_LABEL_OFFSET / m_zoom);
        cr->show_text(label);
    }
}

void Canvas::drawPendingLine(const Cairo::RefPtr<Cairo::Context>& cr)
{
    if (m_currentTool == Tool::Line && m_lineFirstPointSet) {
        cr->set_source_rgba(0.5, 0.5, 0.5, 0.6);
        cr->set_line_width(1.5 / m_zoom);
        std::vector<double> dashes = { 4.0 / m_zoom, 4.0 / m_zoom };
        cr->set_dash(dashes, 0);
        cr->move_to(m_lineFirstX, m_lineFirstY);
        cr->line_to(m_mouseWorldX, m_mouseWorldY);
        cr->stroke();
        cr->unset_dash();
    }
}

// ============================================================
// Hit testing
// ============================================================

std::optional<ElementId> Canvas::hitTest(double wx, double wy) const
{
    double tolerance = HIT_TOLERANCE / m_zoom;

    // Check points first (they're drawn on top)
    for (const auto& [id, elem] : m_elements) {
        if (elem.type == CanvasElementType::Point) {
            double dx = wx - elem.x;
            double dy = wy - elem.y;
            if (dx * dx + dy * dy <= tolerance * tolerance)
                return id;
        }
    }

    // Then check lines
    for (const auto& [id, elem] : m_elements) {
        if (elem.type == CanvasElementType::Line) {
            // Point-to-segment distance
            double lx = elem.x2 - elem.x;
            double ly = elem.y2 - elem.y;
            double lenSq = lx * lx + ly * ly;
            if (lenSq < 1e-12)
                continue;

            double t = ((wx - elem.x) * lx + (wy - elem.y) * ly) / lenSq;
            t = std::clamp(t, 0.0, 1.0);

            double px = elem.x + t * lx;
            double py = elem.y + t * ly;
            double dx = wx - px;
            double dy = wy - py;
            if (dx * dx + dy * dy <= tolerance * tolerance)
                return id;
        }
    }

    return std::nullopt;
}

std::optional<ConstraintId> Canvas::hitTestConstraint(
    double wx, double wy) const
{
    double tolerance = HIT_TOLERANCE / m_zoom;

    for (const auto& [cid, constr] : m_constraints) {
        auto itA = m_elements.find(constr.elementA);
        auto itB = m_elements.find(constr.elementB);
        if (itA == m_elements.end() || itB == m_elements.end())
            continue;

        const auto& elemA = itA->second;
        const auto& elemB = itB->second;

        double ax = elemA.x;
        double ay = elemA.y;
        double bx = elemB.x;
        double by = elemB.y;

        if (elemA.type == CanvasElementType::Line) {
            ax = (elemA.x + elemA.x2) / 2.0;
            ay = (elemA.y + elemA.y2) / 2.0;
        }
        if (elemB.type == CanvasElementType::Line) {
            bx = (elemB.x + elemB.x2) / 2.0;
            by = (elemB.y + elemB.y2) / 2.0;
        }

        // Midpoint of constraint visual
        double mx = (ax + bx) / 2.0;
        double my = (ay + by) / 2.0;

        double dx = wx - mx;
        double dy = wy - my;
        if (dx * dx + dy * dy <= tolerance * tolerance * 4.0)
            return cid;
    }

    return std::nullopt;
}

// ============================================================
// Coordinate transforms
// ============================================================

void Canvas::screenToWorld(double sx, double sy, double& wx, double& wy) const
{
    wx = (sx - m_panX) / m_zoom;
    wy = (sy - m_panY) / m_zoom;
}

void Canvas::worldToScreen(double wx, double wy, double& sx, double& sy) const
{
    sx = wx * m_zoom + m_panX;
    sy = wy * m_zoom + m_panY;
}

// ============================================================
// Event handlers
// ============================================================

void Canvas::onClickPressed([[maybe_unused]] int nPress, double x, double y)
{
    grab_focus();

    double wx = 0.0;
    double wy = 0.0;
    screenToWorld(x, y, wx, wy);

    switch (m_currentTool) {
    case Tool::Select:
        handleSelectClick(wx, wy);
        break;
    case Tool::Point:
        handlePointClick(wx, wy);
        break;
    case Tool::Line:
        handleLineClick(wx, wy);
        break;
    case Tool::DistanceConstraint:
        handleConstraintClick(wx, wy);
        break;
    case Tool::Delete:
        handleDeleteClick(wx, wy);
        break;
    }
}

void Canvas::onClickReleased([[maybe_unused]] int nPress,
    [[maybe_unused]] double x, [[maybe_unused]] double y)
{
    // Nothing needed for now
}

void Canvas::onDragBegin(double startX, double startY)
{
    double wx = 0.0;
    double wy = 0.0;
    screenToWorld(startX, startY, wx, wy);

    if (m_currentTool == Tool::Select) {
        auto hit = hitTest(wx, wy);
        if (hit.has_value() && m_selectedElement == hit) {
            m_isDragging = true;
            m_dragStartWx = wx;
            m_dragStartWy = wy;
            auto& elem = m_elements[*hit];
            m_dragOrigX = elem.x;
            m_dragOrigY = elem.y;
            if (elem.type == CanvasElementType::Line) {
                m_dragOrigX2 = elem.x2;
                m_dragOrigY2 = elem.y2;
            }
            return;
        }
    }

    // Pan with any tool if no element hit
    m_isPanning = true;
    m_dragStartWx = startX; // screen coords for panning
    m_dragStartWy = startY;
    m_dragOrigX = m_panX;
    m_dragOrigY = m_panY;
}

void Canvas::onDragUpdate(double offsetX, double offsetY)
{
    if (m_isDragging && m_selectedElement.has_value()) {
        double dx = offsetX / m_zoom;
        double dy = offsetY / m_zoom;
        auto& elem = m_elements[*m_selectedElement];

        if (elem.type == CanvasElementType::Point) {
            elem.x = m_dragOrigX + dx;
            elem.y = m_dragOrigY + dy;
            m_model.updateElementPosition(*m_selectedElement, elem.x, elem.y);
        } else if (elem.type == CanvasElementType::Line) {
            elem.x = m_dragOrigX + dx;
            elem.y = m_dragOrigY + dy;
            elem.x2 = m_dragOrigX2 + dx;
            elem.y2 = m_dragOrigY2 + dy;
            m_model.updateLinePosition(
                *m_selectedElement, elem.x, elem.y, elem.x2, elem.y2);
        }
        queue_draw();
    } else if (m_isPanning) {
        m_panX = m_dragOrigX + offsetX;
        m_panY = m_dragOrigY + offsetY;
        queue_draw();
    }
}

void Canvas::onDragEnd(
    [[maybe_unused]] double offsetX, [[maybe_unused]] double offsetY)
{
    m_isDragging = false;
    m_isPanning = false;
}

void Canvas::onMotion(double x, double y)
{
    screenToWorld(x, y, m_mouseWorldX, m_mouseWorldY);

    if (m_lineFirstPointSet) {
        queue_draw();
    }

    updateStatus();
}

bool Canvas::onScroll([[maybe_unused]] double dx, double dy)
{
    double factor = (dy < 0) ? ZOOM_FACTOR : (1.0 / ZOOM_FACTOR);
    double newZoom = m_zoom * factor;
    newZoom = std::clamp(newZoom, MIN_ZOOM, MAX_ZOOM);

    // Zoom toward mouse position
    double mouseScreenX = m_mouseWorldX * m_zoom + m_panX;
    double mouseScreenY = m_mouseWorldY * m_zoom + m_panY;

    m_zoom = newZoom;
    m_panX = mouseScreenX - m_mouseWorldX * m_zoom;
    m_panY = mouseScreenY - m_mouseWorldY * m_zoom;

    queue_draw();
    return true;
}

bool Canvas::onKeyPressed(guint keyval, [[maybe_unused]] guint keycode,
    [[maybe_unused]] Gdk::ModifierType state)
{
    if (keyval == GDK_KEY_Delete || keyval == GDK_KEY_BackSpace) {
        if (m_selectedElement.has_value()) {
            auto id = *m_selectedElement;

            // Remove associated constraints from canvas
            auto constraints = m_model.getConstraintsForElement(id);
            for (auto cid : constraints) {
                m_constraints.erase(cid);
            }

            m_model.removeElement(id);
            m_elements.erase(id);
            m_selectedElement.reset();
            updateStatus();
            queue_draw();
            return true;
        }
    }

    if (keyval == GDK_KEY_Escape) {
        clearSelection();
        m_lineFirstPointSet = false;
        m_constraintFirstElement.reset();
        updateStatus();
        return true;
    }

    return false;
}

// ============================================================
// Tool handlers
// ============================================================

void Canvas::handleSelectClick(double wx, double wy)
{
    clearSelection();
    auto hit = hitTest(wx, wy);
    if (hit.has_value()) {
        m_selectedElement = hit;
        m_elements[*hit].selected = true;
    }
    updateStatus();
    queue_draw();
}

void Canvas::handlePointClick(double wx, double wy)
{
    auto eid = m_model.addPoint(wx, wy);
    CanvasElement elem {};
    elem.id = eid;
    elem.type = CanvasElementType::Point;
    elem.x = wx;
    elem.y = wy;
    m_elements[eid] = elem;
    updateStatus();
    queue_draw();
}

void Canvas::handleLineClick(double wx, double wy)
{
    if (!m_lineFirstPointSet) {
        m_lineFirstPointSet = true;
        m_lineFirstX = wx;
        m_lineFirstY = wy;
        updateStatus();
    } else {
        auto eid = m_model.addLine(m_lineFirstX, m_lineFirstY, wx, wy);
        CanvasElement elem {};
        elem.id = eid;
        elem.type = CanvasElementType::Line;
        elem.x = m_lineFirstX;
        elem.y = m_lineFirstY;
        elem.x2 = wx;
        elem.y2 = wy;
        m_elements[eid] = elem;
        m_lineFirstPointSet = false;
        updateStatus();
        queue_draw();
    }
}

void Canvas::handleConstraintClick(double wx, double wy)
{
    auto hit = hitTest(wx, wy);
    if (!hit.has_value())
        return;

    if (!m_constraintFirstElement.has_value()) {
        m_constraintFirstElement = hit;
        // Highlight the first selected element
        clearSelection();
        m_selectedElement = hit;
        m_elements[*hit].selected = true;
        updateStatus();
        queue_draw();
    } else {
        if (*m_constraintFirstElement == *hit) {
            // Can't constrain an element to itself
            return;
        }

        // Emit signal to show constraint dialog
        m_constraintRequestSignal.emit(*m_constraintFirstElement, *hit);

        m_constraintFirstElement.reset();
        clearSelection();
        updateStatus();
    }
}

void Canvas::handleDeleteClick(double wx, double wy)
{
    // First try to hit a constraint
    auto constraintHit = hitTestConstraint(wx, wy);
    if (constraintHit.has_value()) {
        m_model.removeConstraint(*constraintHit);
        m_constraints.erase(*constraintHit);
        updateStatus();
        queue_draw();
        return;
    }

    // Then try to hit an element
    auto elementHit = hitTest(wx, wy);
    if (elementHit.has_value()) {
        auto constraints = m_model.getConstraintsForElement(*elementHit);
        for (auto cid : constraints) {
            m_constraints.erase(cid);
        }

        m_model.removeElement(*elementHit);
        m_elements.erase(*elementHit);
        if (m_selectedElement == elementHit)
            m_selectedElement.reset();
        updateStatus();
        queue_draw();
    }
}

void Canvas::updateStatus()
{
    std::string toolName;
    switch (m_currentTool) {
    case Tool::Select:
        toolName = "Select";
        break;
    case Tool::Point:
        toolName = "Point";
        break;
    case Tool::Line:
        toolName = "Line";
        if (m_lineFirstPointSet)
            toolName += " (click second point)";
        break;
    case Tool::DistanceConstraint:
        toolName = "Distance Constraint";
        if (m_constraintFirstElement.has_value())
            toolName += " (click second element)";
        break;
    case Tool::Delete:
        toolName = "Delete";
        break;
    }

    auto graphInfo = m_model.getStatusText();

    auto status = std::format("Tool: {}  |  Mouse: ({:.0f}, {:.0f})  |  "
                              "Zoom: {:.0f}%  |  {}",
        toolName, m_mouseWorldX, m_mouseWorldY, m_zoom * 100.0, graphInfo);

    m_statusSignal.emit(Glib::ustring(status));
}

} // namespace Gui
