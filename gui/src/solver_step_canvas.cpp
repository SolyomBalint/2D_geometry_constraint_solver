#include "solver_step_canvas.hpp"

// Custom headers
#include "./graph_renderer.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace Gui {

SolverStepCanvas::SolverStepCanvas()
{
    set_hexpand(true);
    set_vexpand(true);

    set_draw_func(sigc::mem_fun(*this, &SolverStepCanvas::onDraw));

    setupControllers();

    // Start centered on origin with a reasonable zoom
    m_zoom = 2.0;
}

void SolverStepCanvas::addSolvedComponent(const Gcs::ConstraintGraph* component)
{
    m_solvedComponents.push_back(component);
    autoFit();
    queue_draw();
}

void SolverStepCanvas::clear()
{
    m_solvedComponents.clear();
    m_panX = 0.0;
    m_panY = 0.0;
    m_zoom = 2.0;
    m_hasAutoFit = false;
    queue_draw();
}

void SolverStepCanvas::autoFit()
{
    // Compute combined bounding box of all solved components
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();

    for (const auto* comp : m_solvedComponents) {
        auto bbox = computeSolverSpaceBoundingBox(*comp);
        if (bbox.width() > 0.0 || bbox.height() > 0.0) {
            minX = std::min(minX, bbox.minX);
            minY = std::min(minY, bbox.minY);
            maxX = std::max(maxX, bbox.maxX);
            maxY = std::max(maxY, bbox.maxY);
        }
    }

    // Always include the origin
    minX = std::min(minX, -10.0);
    minY = std::min(minY, -10.0);
    maxX = std::max(maxX, 10.0);
    maxY = std::max(maxY, 10.0);

    double bboxW = maxX - minX;
    double bboxH = maxY - minY;
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

    double centerX = (minX + maxX) / 2.0;
    double centerY = (minY + maxY) / 2.0;
    m_panX = static_cast<double>(canvasW) / 2.0 - centerX * m_zoom;
    m_panY = static_cast<double>(canvasH) / 2.0 - centerY * m_zoom;

    m_hasAutoFit = true;
}

void SolverStepCanvas::onDraw(
    const Cairo::RefPtr<Cairo::Context>& cr, int width, int height)
{
    // Center on origin if not yet auto-fitted
    if (!m_hasAutoFit) {
        m_panX = static_cast<double>(width) / 2.0;
        m_panY = static_cast<double>(height) / 2.0;
    }

    // White background
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->paint();

    cr->save();
    cr->translate(m_panX, m_panY);
    cr->scale(m_zoom, m_zoom);

    drawGrid(cr, width, height);
    drawOriginMarker(cr);

    // Render previously solved components (dimmed)
    auto componentCount = m_solvedComponents.size();
    for (std::size_t i = 0; i < componentCount; ++i) {
        bool isLatest = (i == componentCount - 1);

        if (isLatest) {
            // Bright accent color for the latest step
            auto color = getComponentColor(static_cast<int>(i));
            renderConstraintGraphSolverSpace(cr, *m_solvedComponents[i], m_zoom,
                color.r, color.g, color.b, false);
        } else {
            // Dimmed gray for previous steps
            renderConstraintGraphSolverSpace(
                cr, *m_solvedComponents[i], m_zoom, 0.5, 0.5, 0.5, true);
        }
    }

    cr->restore();

    // Title label in top-left
    cr->set_source_rgba(0.3, 0.3, 0.3, 0.7);
    cr->set_font_size(12.0);
    cr->move_to(8.0, 20.0);
    cr->show_text("Solver Space Reconstruction");
}

void SolverStepCanvas::setupControllers()
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

void SolverStepCanvas::drawGrid(
    const Cairo::RefPtr<Cairo::Context>& cr, int width, int height)
{
    double baseSpacing = 50.0;
    double spacing = baseSpacing;
    while (spacing * m_zoom < 30.0)
        spacing *= 2.0;
    while (spacing * m_zoom > 120.0)
        spacing /= 2.0;

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

    // Axes (thicker, colored)
    cr->set_line_width(1.5 / m_zoom);

    // X axis (red-ish)
    cr->set_source_rgba(0.7, 0.2, 0.2, 0.6);
    cr->move_to(worldLeft, 0.0);
    cr->line_to(worldRight, 0.0);
    cr->stroke();

    // Y axis (green-ish)
    cr->set_source_rgba(0.2, 0.6, 0.2, 0.6);
    cr->move_to(0.0, worldTop);
    cr->line_to(0.0, worldBottom);
    cr->stroke();
}

void SolverStepCanvas::drawOriginMarker(const Cairo::RefPtr<Cairo::Context>& cr)
{
    double markerSize = 6.0 / m_zoom;

    // Filled circle at origin
    cr->set_source_rgba(0.3, 0.3, 0.3, 0.8);
    cr->arc(0.0, 0.0, markerSize, 0, 2.0 * std::numbers::pi);
    cr->fill();

    // Label
    cr->set_source_rgba(0.3, 0.3, 0.3, 0.8);
    cr->set_font_size(10.0 / m_zoom);
    cr->move_to(markerSize + 2.0 / m_zoom, -markerSize - 2.0 / m_zoom);
    cr->show_text("(0,0)");
}

} // namespace Gui
