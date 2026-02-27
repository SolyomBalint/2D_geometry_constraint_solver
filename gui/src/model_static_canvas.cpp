#include "model_static_canvas.hpp"

// Custom headers
#include "./graph_renderer.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <numbers>

namespace Gui {

ModelStaticCanvas::ModelStaticCanvas()
{
    set_hexpand(true);
    set_vexpand(true);

    set_draw_func(sigc::mem_fun(*this, &ModelStaticCanvas::onDraw));

    setupControllers();
}

void ModelStaticCanvas::setGraph(const Gcs::ConstraintGraph& graph)
{
    m_graph = graph;

    // Auto-fit view to bounding box
    auto bbox = computeBoundingBox(graph);
    double bboxW = bbox.width();
    double bboxH = bbox.height();

    if (bboxW < 1.0)
        bboxW = 100.0;
    if (bboxH < 1.0)
        bboxH = 100.0;

    int canvasW = get_width();
    int canvasH = get_height();
    if (canvasW < 1)
        canvasW = 600;
    if (canvasH < 1)
        canvasH = 400;

    double zoomX = static_cast<double>(canvasW) / (bboxW + 40.0);
    double zoomY = static_cast<double>(canvasH) / (bboxH + 40.0);
    m_zoom = std::min(zoomX, zoomY);

    m_panX = static_cast<double>(canvasW) / 2.0 - bbox.centerX() * m_zoom;
    m_panY = static_cast<double>(canvasH) / 2.0 - bbox.centerY() * m_zoom;

    queue_draw();
}

void ModelStaticCanvas::clear()
{
    m_graph.reset();
    m_panX = 0.0;
    m_panY = 0.0;
    m_zoom = 1.0;
    queue_draw();
}

void ModelStaticCanvas::onDraw(
    const Cairo::RefPtr<Cairo::Context>& cr, int width, int height)
{
    // White background
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->paint();

    cr->save();
    cr->translate(m_panX, m_panY);
    cr->scale(m_zoom, m_zoom);

    drawGrid(cr, width, height);

    if (m_graph.has_value()) {
        renderConstraintGraph(cr, m_graph.value(), m_zoom);
    }

    cr->restore();

    // Title label in top-left
    cr->set_source_rgba(0.3, 0.3, 0.3, 0.7);
    cr->set_font_size(12.0);
    cr->move_to(8.0, 20.0);
    cr->show_text("Original Model (Canvas Space)");
}

void ModelStaticCanvas::setupControllers()
{
    // Pan via drag
    m_dragController = Gtk::GestureDrag::create();
    m_dragController->signal_drag_begin().connect([this](double x, double y) {
        m_dragStartX = x;
        m_dragStartY = y;
        m_dragStartPanX = m_panX;
        m_dragStartPanY = m_panY;
    });
    m_dragController->signal_drag_update().connect(
        [this](double offsetX, double offsetY) {
            m_panX = m_dragStartPanX + offsetX;
            m_panY = m_dragStartPanY + offsetY;
            queue_draw();
        });
    add_controller(m_dragController);

    // Zoom via scroll
    m_scrollController = Gtk::EventControllerScroll::create();
    m_scrollController->set_flags(Gtk::EventControllerScroll::Flags::VERTICAL);
    m_scrollController->signal_scroll().connect(
        [this](double /*dx*/, double dy) -> bool {
            double factor = (dy < 0) ? 1.1 : 1.0 / 1.1;

            // Use center as zoom target
            double mx = static_cast<double>(get_width()) / 2.0;
            double my = static_cast<double>(get_height()) / 2.0;

            double worldX = (mx - m_panX) / m_zoom;
            double worldY = (my - m_panY) / m_zoom;

            m_zoom *= factor;

            m_panX = mx - worldX * m_zoom;
            m_panY = my - worldY * m_zoom;

            queue_draw();
            return true;
        },
        false);
    add_controller(m_scrollController);
}

void ModelStaticCanvas::drawGrid(
    const Cairo::RefPtr<Cairo::Context>& cr, int width, int height)
{
    // Determine grid spacing based on zoom
    double baseSpacing = 50.0;
    double spacing = baseSpacing;
    while (spacing * m_zoom < 30.0)
        spacing *= 2.0;
    while (spacing * m_zoom > 120.0)
        spacing /= 2.0;

    // Compute visible world bounds
    double worldLeft = -m_panX / m_zoom;
    double worldTop = -m_panY / m_zoom;
    double worldRight = (static_cast<double>(width) - m_panX) / m_zoom;
    double worldBottom = (static_cast<double>(height) - m_panY) / m_zoom;

    double startX = std::floor(worldLeft / spacing) * spacing;
    double startY = std::floor(worldTop / spacing) * spacing;

    // Grid lines
    cr->set_source_rgba(0.85, 0.85, 0.85, 0.5);
    cr->set_line_width(0.5 / m_zoom);

    for (double x = startX; x <= worldRight; x += spacing) {
        cr->move_to(x, worldTop);
        cr->line_to(x, worldBottom);
    }
    for (double y = startY; y <= worldBottom; y += spacing) {
        cr->move_to(worldLeft, y);
        cr->line_to(worldRight, y);
    }
    cr->stroke();

    // Axes
    cr->set_source_rgba(0.5, 0.5, 0.5, 0.6);
    cr->set_line_width(1.0 / m_zoom);

    cr->move_to(worldLeft, 0.0);
    cr->line_to(worldRight, 0.0);
    cr->move_to(0.0, worldTop);
    cr->line_to(0.0, worldBottom);
    cr->stroke();
}

} // namespace Gui
