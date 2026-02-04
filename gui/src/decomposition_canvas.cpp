#include "decomposition_canvas.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

namespace Gui {

namespace {
    constexpr double ZOOM_FACTOR = 1.1;
    constexpr double MIN_ZOOM = 0.05;
    constexpr double MAX_ZOOM = 20.0;
    constexpr double GRID_SPACING = 50.0;
} // namespace

DecompositionCanvas::DecompositionCanvas()
{
    set_hexpand(true);
    set_vexpand(true);

    set_draw_func(sigc::mem_fun(*this, &DecompositionCanvas::onDraw));

    // Drag gesture for panning
    m_dragGesture = Gtk::GestureDrag::create();
    m_dragGesture->set_button(GDK_BUTTON_PRIMARY);
    m_dragGesture->signal_drag_begin().connect(
        sigc::mem_fun(*this, &DecompositionCanvas::onDragBegin));
    m_dragGesture->signal_drag_update().connect(
        sigc::mem_fun(*this, &DecompositionCanvas::onDragUpdate));
    m_dragGesture->signal_drag_end().connect(
        sigc::mem_fun(*this, &DecompositionCanvas::onDragEnd));
    add_controller(m_dragGesture);

    // Scroll controller for zoom
    m_scrollController = Gtk::EventControllerScroll::create();
    m_scrollController->set_flags(Gtk::EventControllerScroll::Flags::VERTICAL);
    m_scrollController->signal_scroll().connect(
        sigc::mem_fun(*this, &DecompositionCanvas::onScroll), false);
    add_controller(m_scrollController);

    // Motion controller for tracking mouse position
    m_motionController = Gtk::EventControllerMotion::create();
    m_motionController->signal_motion().connect(
        sigc::mem_fun(*this, &DecompositionCanvas::onMotion));
    add_controller(m_motionController);
}

void DecompositionCanvas::setDecomposition(
    const MathUtils::BinaryTree<Gcs::ConstraintGraph>& stree)
{
    m_leafGraphs = stree.getLeafValues();

    // Compute bounding box of all leaves and center the view
    if (!m_leafGraphs.empty()) {
        double globalMinX = std::numeric_limits<double>::max();
        double globalMinY = std::numeric_limits<double>::max();
        double globalMaxX = std::numeric_limits<double>::lowest();
        double globalMaxY = std::numeric_limits<double>::lowest();

        for (const auto& leaf : m_leafGraphs) {
            auto bbox = computeBoundingBox(leaf);
            globalMinX = std::min(globalMinX, bbox.minX);
            globalMinY = std::min(globalMinY, bbox.minY);
            globalMaxX = std::max(globalMaxX, bbox.maxX);
            globalMaxY = std::max(globalMaxY, bbox.maxY);
        }

        // Center the view on the combined bounding box
        double centerX = (globalMinX + globalMaxX) / 2.0;
        double centerY = (globalMinY + globalMaxY) / 2.0;

        int width = get_width();
        int height = get_height();
        if (width <= 0)
            width = 800;
        if (height <= 0)
            height = 600;

        // Calculate zoom to fit
        double rangeX = globalMaxX - globalMinX;
        double rangeY = globalMaxY - globalMinY;
        if (rangeX > 0 && rangeY > 0) {
            double zoomX
                = static_cast<double>(width) / rangeX * 0.9; // 90% fill
            double zoomY = static_cast<double>(height) / rangeY * 0.9;
            m_zoom = std::min(zoomX, zoomY);
            m_zoom = std::clamp(m_zoom, MIN_ZOOM, MAX_ZOOM);
        } else {
            m_zoom = 1.0;
        }

        m_panX = static_cast<double>(width) / 2.0 - centerX * m_zoom;
        m_panY = static_cast<double>(height) / 2.0 - centerY * m_zoom;
    }

    queue_draw();
}

void DecompositionCanvas::clear()
{
    m_leafGraphs.clear();
    m_panX = 0.0;
    m_panY = 0.0;
    m_zoom = 1.0;
    queue_draw();
}

void DecompositionCanvas::onDraw(
    const Cairo::RefPtr<Cairo::Context>& cr, int width, int height)
{
    // White background
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->paint();

    cr->save();
    cr->translate(m_panX, m_panY);
    cr->scale(m_zoom, m_zoom);

    drawGrid(cr, width, height);

    // Draw each leaf component with a distinct color
    for (int i = 0; i < static_cast<int>(m_leafGraphs.size()); ++i) {
        auto color = getComponentColor(i);
        renderConstraintGraph(cr, m_leafGraphs[static_cast<std::size_t>(i)],
            m_zoom, color.r, color.g, color.b);
    }

    cr->restore();

    // Draw legend in top-left corner
    if (!m_leafGraphs.empty()) {
        cr->set_font_size(12.0);
        double legendX = 10.0;
        double legendY = 20.0;

        for (int i = 0; i < static_cast<int>(m_leafGraphs.size()); ++i) {
            auto color = getComponentColor(i);
            const auto& leaf = m_leafGraphs[static_cast<std::size_t>(i)];

            // Color swatch
            cr->set_source_rgb(color.r, color.g, color.b);
            cr->rectangle(legendX, legendY - 10.0, 14.0, 14.0);
            cr->fill();

            // Border
            cr->set_source_rgb(0.0, 0.0, 0.0);
            cr->set_line_width(1.0);
            cr->rectangle(legendX, legendY - 10.0, 14.0, 14.0);
            cr->stroke();

            // Label
            cr->set_source_rgb(0.0, 0.0, 0.0);
            std::string label = std::format("Component {} (N:{} E:{})", i + 1,
                leaf.nodeCount(), leaf.edgeCount());
            bool hasVirtual = leaf.hasVirtualEdge();
            if (hasVirtual) {
                label += std::format(" [V:{}]", leaf.getVirtualEdges().size());
            }
            cr->move_to(legendX + 20.0, legendY);
            cr->show_text(label);

            legendY += 20.0;
        }
    }
}

void DecompositionCanvas::drawGrid(
    const Cairo::RefPtr<Cairo::Context>& cr, int width, int height)
{
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    screenToWorld(0, 0, x0, y0);
    screenToWorld(
        static_cast<double>(width), static_cast<double>(height), x1, y1);

    double gridStep = GRID_SPACING;
    while (gridStep * m_zoom < 20.0)
        gridStep *= 2.0;
    while (gridStep * m_zoom > 100.0)
        gridStep /= 2.0;

    double startX = std::floor(x0 / gridStep) * gridStep;
    double startY = std::floor(y0 / gridStep) * gridStep;

    cr->set_line_width(0.5 / m_zoom);
    cr->set_source_rgba(0.90, 0.90, 0.90, 1.0);

    for (double gx = startX; gx <= x1; gx += gridStep) {
        cr->move_to(gx, y0);
        cr->line_to(gx, y1);
    }
    for (double gy = startY; gy <= y1; gy += gridStep) {
        cr->move_to(x0, gy);
        cr->line_to(x1, gy);
    }
    cr->stroke();

    // Axes
    cr->set_source_rgba(0.7, 0.7, 0.7, 1.0);
    cr->set_line_width(1.0 / m_zoom);
    cr->move_to(x0, 0);
    cr->line_to(x1, 0);
    cr->stroke();
    cr->move_to(0, y0);
    cr->line_to(0, y1);
    cr->stroke();
}

void DecompositionCanvas::screenToWorld(
    double sx, double sy, double& wx, double& wy) const
{
    wx = (sx - m_panX) / m_zoom;
    wy = (sy - m_panY) / m_zoom;
}

void DecompositionCanvas::onDragBegin(double startX, double startY)
{
    m_dragStartX = startX;
    m_dragStartY = startY;
    m_dragOrigPanX = m_panX;
    m_dragOrigPanY = m_panY;
}

void DecompositionCanvas::onDragUpdate(double offsetX, double offsetY)
{
    m_panX = m_dragOrigPanX + offsetX;
    m_panY = m_dragOrigPanY + offsetY;
    queue_draw();
}

void DecompositionCanvas::onDragEnd(
    [[maybe_unused]] double offsetX, [[maybe_unused]] double offsetY)
{
    // Nothing needed
}

bool DecompositionCanvas::onScroll([[maybe_unused]] double dx, double dy)
{
    double factor = (dy < 0) ? ZOOM_FACTOR : (1.0 / ZOOM_FACTOR);
    double newZoom = m_zoom * factor;
    newZoom = std::clamp(newZoom, MIN_ZOOM, MAX_ZOOM);

    // Zoom toward mouse position
    double mouseWorldX = (m_mouseScreenX - m_panX) / m_zoom;
    double mouseWorldY = (m_mouseScreenY - m_panY) / m_zoom;

    m_zoom = newZoom;
    m_panX = m_mouseScreenX - mouseWorldX * m_zoom;
    m_panY = m_mouseScreenY - mouseWorldY * m_zoom;

    queue_draw();
    return true;
}

void DecompositionCanvas::onMotion(double x, double y)
{
    m_mouseScreenX = x;
    m_mouseScreenY = y;
}

} // namespace Gui
