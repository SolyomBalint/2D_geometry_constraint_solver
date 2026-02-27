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
    constexpr double ANGLE_ARC_RADIUS = 30.0;
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
    m_angleConstraintFirstElement.reset();
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

Canvas::AngleConstraintRequestSignal Canvas::signalAngleConstraintRequested()
{
    return m_angleConstraintRequestSignal;
}

void Canvas::addCanvasElement(const CanvasElement& element)
{
    m_elements[element.id] = element;
    queue_draw();
}

void Canvas::addCanvasConstraint(const CanvasConstraint& constraint)
{
    m_constraints[constraint.id] = constraint;
    queue_draw();
}

void Canvas::clearAll()
{
    m_elements.clear();
    m_constraints.clear();
    m_selectedElement.reset();
    m_constraintFirstElement.reset();
    m_angleConstraintFirstElement.reset();
    m_lineFirstPointSet = false;
    m_isDragging = false;
    m_isPanning = false;
    m_pendingAnglePlacement.reset();
    queue_draw();
}

double Canvas::getPanX() const
{
    return m_panX;
}

double Canvas::getPanY() const
{
    return m_panY;
}

double Canvas::getZoom() const
{
    return m_zoom;
}

void Canvas::setPanZoom(double panX, double panY, double zoom)
{
    m_panX = panX;
    m_panY = panY;
    m_zoom = zoom;
    queue_draw();
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
    drawAnglePlacementPreview(cr);

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
    double fontSize = 11.0 / m_zoom;

    for (const auto& [constraintId, constraint] : m_constraints) {
        auto iteratorA = m_elements.find(constraint.elementA);
        auto iteratorB = m_elements.find(constraint.elementB);
        if (iteratorA == m_elements.end() || iteratorB == m_elements.end()) {
            continue;
        }

        const auto& elementA = iteratorA->second;
        const auto& elementB = iteratorB->second;

        if (constraint.type == CanvasConstraintType::Angle) {
            // Use value-matching only after both elements have been
            // solved; before solving, respect the user's placement.
            bool solved = m_model.isElementSolved(constraint.elementA)
                && m_model.isElementSolved(constraint.elementB);
            drawAngleConstraint(
                cr, constraint, elementA, elementB, fontSize, solved);
        } else {
            drawDistanceConstraint(
                cr, constraint, elementA, elementB, fontSize);
        }
    }
}

void Canvas::drawDistanceConstraint(const Cairo::RefPtr<Cairo::Context>& cr,
    const CanvasConstraint& constraint, const CanvasElement& elementA,
    const CanvasElement& elementB, double fontSize)
{
    double lineWidth = 1.5 / m_zoom;

    // Project a point onto the infinite line through (x1,y1)-(x2,y2)
    // and return the perpendicular foot.
    auto projectOntoLine
        = [](double px, double py, double x1, double y1, double x2, double y2,
              double& footX, double& footY) {
              double dx = x2 - x1;
              double dy = y2 - y1;
              double lengthSq = dx * dx + dy * dy;
              if (lengthSq < 1e-12) {
                  footX = x1;
                  footY = y1;
                  return;
              }
              double t = ((px - x1) * dx + (py - y1) * dy) / lengthSq;
              footX = x1 + t * dx;
              footY = y1 + t * dy;
          };

    // Compute endpoint for each element.  For a line paired with a
    // point the endpoint is the perpendicular foot of the point on
    // the line (not the line midpoint), because the distance
    // constraint represents perpendicular distance.
    double centerAX = elementA.x;
    double centerAY = elementA.y;
    double centerBX = elementB.x;
    double centerBY = elementB.y;

    if (elementA.type == CanvasElementType::Line
        && elementB.type == CanvasElementType::Point) {
        projectOntoLine(elementB.x, elementB.y, elementA.x, elementA.y,
            elementA.x2, elementA.y2, centerAX, centerAY);
    } else if (elementA.type == CanvasElementType::Line) {
        centerAX = (elementA.x + elementA.x2) / 2.0;
        centerAY = (elementA.y + elementA.y2) / 2.0;
    }

    if (elementB.type == CanvasElementType::Line
        && elementA.type == CanvasElementType::Point) {
        projectOntoLine(elementA.x, elementA.y, elementB.x, elementB.y,
            elementB.x2, elementB.y2, centerBX, centerBY);
    } else if (elementB.type == CanvasElementType::Line) {
        centerBX = (elementB.x + elementB.x2) / 2.0;
        centerBY = (elementB.y + elementB.y2) / 2.0;
    }

    // Draw dashed line between elements
    cr->set_source_rgba(0.8, 0.2, 0.2, 0.7);
    cr->set_line_width(lineWidth);
    std::vector<double> dashes = { 6.0 / m_zoom, 4.0 / m_zoom };
    cr->set_dash(dashes, 0);
    cr->move_to(centerAX, centerAY);
    cr->line_to(centerBX, centerBY);
    cr->stroke();
    cr->unset_dash();

    // Draw distance label at midpoint
    double midpointX = (centerAX + centerBX) / 2.0;
    double midpointY = (centerAY + centerBY) / 2.0;

    cr->set_source_rgb(0.8, 0.1, 0.1);
    cr->set_font_size(fontSize);

    std::string label = std::format("{:.1f}", constraint.value);
    Cairo::TextExtents extents;
    cr->get_text_extents(label, extents);

    cr->move_to(midpointX - extents.width / 2.0,
        midpointY - CONSTRAINT_LABEL_OFFSET / m_zoom);
    cr->show_text(label);
}

void Canvas::drawAngleConstraint(const Cairo::RefPtr<Cairo::Context>& cr,
    const CanvasConstraint& constraint, const CanvasElement& lineA,
    const CanvasElement& lineB, double fontSize, bool useValueMatching)
{
    // Compute the intersection point of the two lines.
    // Line A: from (lineA.x, lineA.y) to (lineA.x2, lineA.y2)
    // Line B: from (lineB.x, lineB.y) to (lineB.x2, lineB.y2)
    double directionAX = lineA.x2 - lineA.x;
    double directionAY = lineA.y2 - lineA.y;
    double directionBX = lineB.x2 - lineB.x;
    double directionBY = lineB.y2 - lineB.y;

    // Solve for intersection using parametric form:
    //   P = lineA.p1 + t * dirA
    //   P = lineB.p1 + s * dirB
    // Cross product of directions (determinant of 2x2 matrix)
    double crossProduct = directionAX * directionBY - directionAY * directionBX;

    double intersectionX = 0.0;
    double intersectionY = 0.0;

    if (std::abs(crossProduct) < 1e-9) {
        // Lines are nearly parallel — fall back to midpoint of
        // the two line midpoints.
        intersectionX = (lineA.x + lineA.x2 + lineB.x + lineB.x2) / 4.0;
        intersectionY = (lineA.y + lineA.y2 + lineB.y + lineB.y2) / 4.0;
    } else {
        // Parametric t for the intersection on line A
        double deltaOriginX = lineB.x - lineA.x;
        double deltaOriginY = lineB.y - lineA.y;
        double parameterT
            = (deltaOriginX * directionBY - deltaOriginY * directionBX)
            / crossProduct;
        intersectionX = lineA.x + parameterT * directionAX;
        intersectionY = lineA.y + parameterT * directionAY;
    }

    // During placement preview, use the flipped flag directly
    // (the user is choosing which side via mouse position).
    // For committed constraints, use value-matching: compare the
    // arc angle to the constraint value and flip to the side
    // that matches, since post-solve directions may have changed.
    if (useValueMatching) {
        // Compute default arc and check against constraint value.
        double tempAngleA = std::atan2(directionAY, directionAX);
        double tempDiff = std::atan2(directionBY, directionBX) - tempAngleA;
        while (tempDiff > std::numbers::pi) {
            tempDiff -= 2.0 * std::numbers::pi;
        }
        while (tempDiff < -std::numbers::pi) {
            tempDiff += 2.0 * std::numbers::pi;
        }
        double constraintRadians = constraint.value * std::numbers::pi / 180.0;
        double defaultArcAngle = std::abs(tempDiff);
        double supplementaryArcAngle = std::numbers::pi - defaultArcAngle;
        if (std::abs(supplementaryArcAngle - constraintRadians)
            < std::abs(defaultArcAngle - constraintRadians)) {
            directionAX = -directionAX;
            directionAY = -directionAY;
        }
    } else if (constraint.flipped) {
        directionAX = -directionAX;
        directionAY = -directionAY;
    }

    // Compute the angle of each line direction relative to X axis.
    double angleA = std::atan2(directionAY, directionAX);
    double angleB = std::atan2(directionBY, directionBX);

    // Normalize the sweep to [-pi, pi] (the smaller angle).
    double angleDifference = angleB - angleA;
    while (angleDifference > std::numbers::pi) {
        angleDifference -= 2.0 * std::numbers::pi;
    }
    while (angleDifference < -std::numbers::pi) {
        angleDifference += 2.0 * std::numbers::pi;
    }

    double arcStartAngle = angleA;
    double arcEndAngle = angleA + angleDifference;

    // Draw the arc
    double arcRadius = ANGLE_ARC_RADIUS / m_zoom;
    cr->set_source_rgba(0.1, 0.5, 0.8, 0.8);
    cr->set_line_width(1.5 / m_zoom);

    // begin_new_sub_path() clears any leftover current point so that
    // Cairo does not draw a spurious straight line from a previous
    // drawing operation (e.g. show_text) to the arc start.
    cr->begin_new_sub_path();
    if (angleDifference >= 0.0) {
        cr->arc(intersectionX, intersectionY, arcRadius, arcStartAngle,
            arcEndAngle);
    } else {
        cr->arc_negative(intersectionX, intersectionY, arcRadius, arcStartAngle,
            arcEndAngle);
    }
    cr->stroke();

    // Draw small lines from the intersection outward along each
    // line direction to visually anchor the arc. Orient each tick
    // toward the line segment (not blindly in the p2-p1 direction)
    // so it overlays on the visible geometry.
    double tickLength = arcRadius * 1.3;
    cr->set_source_rgba(0.1, 0.5, 0.8, 0.5);
    cr->set_line_width(1.0 / m_zoom);

    double lengthA = std::hypot(directionAX, directionAY);
    double lengthB = std::hypot(directionBX, directionBY);

    if (lengthA > 1e-9) {
        double tickAX = directionAX / lengthA;
        double tickAY = directionAY / lengthA;
        double midAX = (lineA.x + lineA.x2) / 2.0;
        double midAY = (lineA.y + lineA.y2) / 2.0;
        if ((midAX - intersectionX) * tickAX + (midAY - intersectionY) * tickAY
            < 0.0) {
            tickAX = -tickAX;
            tickAY = -tickAY;
        }
        cr->move_to(intersectionX, intersectionY);
        cr->line_to(intersectionX + tickAX * tickLength,
            intersectionY + tickAY * tickLength);
        cr->stroke();
    }
    if (lengthB > 1e-9) {
        double tickBX = directionBX / lengthB;
        double tickBY = directionBY / lengthB;
        double midBX = (lineB.x + lineB.x2) / 2.0;
        double midBY = (lineB.y + lineB.y2) / 2.0;
        if ((midBX - intersectionX) * tickBX + (midBY - intersectionY) * tickBY
            < 0.0) {
            tickBX = -tickBX;
            tickBY = -tickBY;
        }
        cr->move_to(intersectionX, intersectionY);
        cr->line_to(intersectionX + tickBX * tickLength,
            intersectionY + tickBY * tickLength);
        cr->stroke();
    }

    // Draw the angle label at the midpoint of the arc
    double labelAngle = arcStartAngle + angleDifference / 2.0;
    double labelRadius = arcRadius + CONSTRAINT_LABEL_OFFSET / m_zoom;
    double labelX = intersectionX + labelRadius * std::cos(labelAngle);
    double labelY = intersectionY + labelRadius * std::sin(labelAngle);

    cr->set_source_rgb(0.1, 0.4, 0.7);
    cr->set_font_size(fontSize);

    // Display the angle in degrees with degree symbol
    std::string label = std::format("{:.1f}\u00B0", constraint.value);
    Cairo::TextExtents extents;
    cr->get_text_extents(label, extents);

    cr->move_to(labelX - extents.width / 2.0, labelY + extents.height / 2.0);
    cr->show_text(label);
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

void Canvas::drawAnglePlacementPreview(const Cairo::RefPtr<Cairo::Context>& cr)
{
    if (!m_pendingAnglePlacement.has_value()) {
        return;
    }

    const auto& pending = *m_pendingAnglePlacement;
    auto iterA = m_elements.find(pending.elementA);
    auto iterB = m_elements.find(pending.elementB);
    if (iterA == m_elements.end() || iterB == m_elements.end()) {
        return;
    }

    // Build a temporary CanvasConstraint to reuse drawAngleConstraint.
    CanvasConstraint preview {};
    preview.elementA = pending.elementA;
    preview.elementB = pending.elementB;
    preview.value = pending.angleDegrees;
    preview.type = CanvasConstraintType::Angle;
    preview.flipped = pending.flipped;

    double fontSize = 11.0 / m_zoom;
    // Use flipped flag directly (not value-matching) for placement.
    drawAngleConstraint(
        cr, preview, iterA->second, iterB->second, fontSize, false);
}

void Canvas::startAnglePlacement(
    ElementId elemA, ElementId elemB, double angleDegrees)
{
    m_pendingAnglePlacement = PendingAnglePlacement {
        .elementA = elemA,
        .elementB = elemB,
        .angleDegrees = angleDegrees,
        .flipped = false,
    };
    clearSelection();
    updateStatus();
    queue_draw();
}

Canvas::AngleConstraintConfirmedSignal Canvas::signalAngleConstraintConfirmed()
{
    return m_angleConstraintConfirmedSignal;
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

    for (const auto& [constraintId, constraint] : m_constraints) {
        auto iteratorA = m_elements.find(constraint.elementA);
        auto iteratorB = m_elements.find(constraint.elementB);
        if (iteratorA == m_elements.end() || iteratorB == m_elements.end()) {
            continue;
        }

        const auto& elementA = iteratorA->second;
        const auto& elementB = iteratorB->second;

        double hitPointX = 0.0;
        double hitPointY = 0.0;

        if (constraint.type == CanvasConstraintType::Angle) {
            // For angle constraints, hit-test near the arc label
            // position: the midpoint of the arc at the intersection
            // of the two lines.
            double directionAX = elementA.x2 - elementA.x;
            double directionAY = elementA.y2 - elementA.y;
            double directionBX = elementB.x2 - elementB.x;
            double directionBY = elementB.y2 - elementB.y;

            double crossProduct
                = directionAX * directionBY - directionAY * directionBX;

            double intersectionX = 0.0;
            double intersectionY = 0.0;

            if (std::abs(crossProduct) < 1e-9) {
                intersectionX
                    = (elementA.x + elementA.x2 + elementB.x + elementB.x2)
                    / 4.0;
                intersectionY
                    = (elementA.y + elementA.y2 + elementB.y + elementB.y2)
                    / 4.0;
            } else {
                double deltaOriginX = elementB.x - elementA.x;
                double deltaOriginY = elementB.y - elementA.y;
                double parameterT
                    = (deltaOriginX * directionBY - deltaOriginY * directionBX)
                    / crossProduct;
                intersectionX = elementA.x + parameterT * directionAX;
                intersectionY = elementA.y + parameterT * directionAY;
            }

            // Match the flip logic from drawAngleConstraint:
            // value-matching only when both elements are solved.
            bool solved = m_model.isElementSolved(constraint.elementA)
                && m_model.isElementSolved(constraint.elementB);
            if (solved) {
                double tempAngleA = std::atan2(directionAY, directionAX);
                double tempDiff
                    = std::atan2(directionBY, directionBX) - tempAngleA;
                while (tempDiff > std::numbers::pi) {
                    tempDiff -= 2.0 * std::numbers::pi;
                }
                while (tempDiff < -std::numbers::pi) {
                    tempDiff += 2.0 * std::numbers::pi;
                }
                double constraintRad
                    = constraint.value * std::numbers::pi / 180.0;
                double defArc = std::abs(tempDiff);
                double supArc = std::numbers::pi - defArc;
                if (std::abs(supArc - constraintRad)
                    < std::abs(defArc - constraintRad)) {
                    directionAX = -directionAX;
                    directionAY = -directionAY;
                }
            } else if (constraint.flipped) {
                directionAX = -directionAX;
                directionAY = -directionAY;
            }

            double angleA = std::atan2(directionAY, directionAX);
            double angleB = std::atan2(directionBY, directionBX);
            double angleDifference = angleB - angleA;
            while (angleDifference > std::numbers::pi) {
                angleDifference -= 2.0 * std::numbers::pi;
            }
            while (angleDifference < -std::numbers::pi) {
                angleDifference += 2.0 * std::numbers::pi;
            }

            double labelAngle = angleA + angleDifference / 2.0;
            double labelRadius
                = ANGLE_ARC_RADIUS / m_zoom + CONSTRAINT_LABEL_OFFSET / m_zoom;
            hitPointX = intersectionX + labelRadius * std::cos(labelAngle);
            hitPointY = intersectionY + labelRadius * std::sin(labelAngle);
        } else {
            // For distance constraints, hit-test at the midpoint of
            // the perpendicular dashed line between elements.
            auto projectOntoLine
                = [](double px, double py, double x1, double y1, double x2,
                      double y2, double& footX, double& footY) {
                      double dx = x2 - x1;
                      double dy = y2 - y1;
                      double lengthSq = dx * dx + dy * dy;
                      if (lengthSq < 1e-12) {
                          footX = x1;
                          footY = y1;
                          return;
                      }
                      double t = ((px - x1) * dx + (py - y1) * dy) / lengthSq;
                      footX = x1 + t * dx;
                      footY = y1 + t * dy;
                  };

            double centerAX = elementA.x;
            double centerAY = elementA.y;
            double centerBX = elementB.x;
            double centerBY = elementB.y;

            if (elementA.type == CanvasElementType::Line
                && elementB.type == CanvasElementType::Point) {
                projectOntoLine(elementB.x, elementB.y, elementA.x, elementA.y,
                    elementA.x2, elementA.y2, centerAX, centerAY);
            } else if (elementA.type == CanvasElementType::Line) {
                centerAX = (elementA.x + elementA.x2) / 2.0;
                centerAY = (elementA.y + elementA.y2) / 2.0;
            }

            if (elementB.type == CanvasElementType::Line
                && elementA.type == CanvasElementType::Point) {
                projectOntoLine(elementA.x, elementA.y, elementB.x, elementB.y,
                    elementB.x2, elementB.y2, centerBX, centerBY);
            } else if (elementB.type == CanvasElementType::Line) {
                centerBX = (elementB.x + elementB.x2) / 2.0;
                centerBY = (elementB.y + elementB.y2) / 2.0;
            }

            hitPointX = (centerAX + centerBX) / 2.0;
            hitPointY = (centerAY + centerBY) / 2.0;
        }

        double deltaX = wx - hitPointX;
        double deltaY = wy - hitPointY;
        if (deltaX * deltaX + deltaY * deltaY <= tolerance * tolerance * 4.0) {
            return constraintId;
        }
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

    // If in angle placement mode, confirm the placement.
    if (m_pendingAnglePlacement.has_value()) {
        const auto& pending = *m_pendingAnglePlacement;
        m_angleConstraintConfirmedSignal.emit(pending.elementA,
            pending.elementB, pending.angleDegrees, pending.flipped);
        m_pendingAnglePlacement.reset();
        updateStatus();
        queue_draw();
        return;
    }

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
    case Tool::AngleConstraint:
        handleAngleConstraintClick(wx, wy);
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

    // During angle placement, determine which side the mouse is
    // on and update the flipped state accordingly.
    if (m_pendingAnglePlacement.has_value()) {
        auto& pending = *m_pendingAnglePlacement;
        auto iterA = m_elements.find(pending.elementA);
        auto iterB = m_elements.find(pending.elementB);
        if (iterA != m_elements.end() && iterB != m_elements.end()) {
            const auto& lineA = iterA->second;
            const auto& lineB = iterB->second;

            // Compute intersection of the two lines.
            double dirAX = lineA.x2 - lineA.x;
            double dirAY = lineA.y2 - lineA.y;
            double dirBX = lineB.x2 - lineB.x;
            double dirBY = lineB.y2 - lineB.y;
            double cross = dirAX * dirBY - dirAY * dirBX;

            double ix = 0.0;
            double iy = 0.0;
            if (std::abs(cross) < 1e-9) {
                ix = (lineA.x + lineA.x2 + lineB.x + lineB.x2) / 4.0;
                iy = (lineA.y + lineA.y2 + lineB.y + lineB.y2) / 4.0;
            } else {
                double dox = lineB.x - lineA.x;
                double doy = lineB.y - lineA.y;
                double t = (dox * dirBY - doy * dirBX) / cross;
                ix = lineA.x + t * dirAX;
                iy = lineA.y + t * dirAY;
            }

            // Compute angle from intersection to mouse position.
            double mouseAngle
                = std::atan2(m_mouseWorldY - iy, m_mouseWorldX - ix);

            // Compute the two possible arc midpoint angles (default
            // and flipped) and pick whichever is closer to the mouse.
            double angleA = std::atan2(dirAY, dirAX);
            double angleB = std::atan2(dirBY, dirBX);
            double diff = angleB - angleA;
            while (diff > std::numbers::pi) {
                diff -= 2.0 * std::numbers::pi;
            }
            while (diff < -std::numbers::pi) {
                diff += 2.0 * std::numbers::pi;
            }
            double defaultMid = angleA + diff / 2.0;

            // Flipped: negate dirA → angleA shifts by pi.
            double flippedAngleA = angleA + std::numbers::pi;
            double flippedDiff = angleB - flippedAngleA;
            while (flippedDiff > std::numbers::pi) {
                flippedDiff -= 2.0 * std::numbers::pi;
            }
            while (flippedDiff < -std::numbers::pi) {
                flippedDiff += 2.0 * std::numbers::pi;
            }
            double flippedMid = flippedAngleA + flippedDiff / 2.0;

            // Angular distance from mouse to each midpoint.
            auto angleDist = [](double a, double b) {
                double d = a - b;
                while (d > std::numbers::pi) {
                    d -= 2.0 * std::numbers::pi;
                }
                while (d < -std::numbers::pi) {
                    d += 2.0 * std::numbers::pi;
                }
                return std::abs(d);
            };

            pending.flipped = angleDist(mouseAngle, flippedMid)
                < angleDist(mouseAngle, defaultMid);
        }
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
        m_angleConstraintFirstElement.reset();
        m_pendingAnglePlacement.reset();
        updateStatus();
        queue_draw();
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

void Canvas::handleAngleConstraintClick(double wx, double wy)
{
    auto hit = hitTest(wx, wy);
    if (!hit.has_value()) {
        return;
    }

    // Only accept Line elements for angle constraints.
    auto elementIterator = m_elements.find(*hit);
    if (elementIterator == m_elements.end()
        || elementIterator->second.type != CanvasElementType::Line) {
        return;
    }

    if (!m_angleConstraintFirstElement.has_value()) {
        m_angleConstraintFirstElement = hit;
        // Highlight the first selected line
        clearSelection();
        m_selectedElement = hit;
        m_elements[*hit].selected = true;
        updateStatus();
        queue_draw();
    } else {
        if (*m_angleConstraintFirstElement == *hit) {
            // Can't constrain a line to itself
            return;
        }

        // Emit signal to show angle constraint dialog
        m_angleConstraintRequestSignal.emit(
            *m_angleConstraintFirstElement, *hit);

        m_angleConstraintFirstElement.reset();
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
    case Tool::AngleConstraint:
        toolName = "Angle Constraint";
        if (m_pendingAnglePlacement.has_value())
            toolName += " (click to place angle arc)";
        else if (m_angleConstraintFirstElement.has_value())
            toolName += " (click second line)";
        else
            toolName += " (click first line)";
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
