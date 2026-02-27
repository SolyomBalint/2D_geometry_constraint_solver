#include "decomposition_canvas.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>

namespace Gui {

namespace {
    constexpr double ZOOM_FACTOR = 1.1;
    constexpr double MIN_ZOOM = 0.05;
    constexpr double MAX_ZOOM = 20.0;

    // Grid cell constants for tiling leaf components
    constexpr double CELL_PADDING = 20.0;
    constexpr double CELL_HEADER_HEIGHT = 24.0;
    constexpr double CELL_GAP = 15.0;
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

    // Build original-ID map from the root graph so that node labels
    // in every subgraph show the root-level IDs.
    auto root = stree.getRoot();
    if (root.has_value()) {
        m_originalIds = buildOriginalIdMap(stree.getValue(*root));
    } else {
        m_originalIds.clear();
    }

    if (!m_leafGraphs.empty()) {
        // Compute grid layout dimensions
        auto leafCount = static_cast<int>(m_leafGraphs.size());
        int cols = static_cast<int>(
            std::ceil(std::sqrt(static_cast<double>(leafCount))));
        int rows = (leafCount + cols - 1) / cols;

        // Determine cell size from the largest component bbox
        double maxCellContent = 0.0;
        for (const auto& leaf : m_leafGraphs) {
            auto bbox = computeAbstractBoundingBox(leaf);
            maxCellContent = std::max(
                maxCellContent, std::max(bbox.width(), bbox.height()));
        }

        double cellSize
            = maxCellContent + 2.0 * CELL_PADDING + CELL_HEADER_HEIGHT;
        cellSize = std::max(cellSize, 200.0); // Minimum cell size

        // Compute total grid bounding box
        double totalWidth = static_cast<double>(cols) * cellSize
            + static_cast<double>(cols - 1) * CELL_GAP;
        double totalHeight = static_cast<double>(rows) * cellSize
            + static_cast<double>(rows - 1) * CELL_GAP;

        // Store layout info for drawing
        m_cellSize = cellSize;
        m_gridCols = cols;

        // Center the view on the grid
        double centerX = totalWidth / 2.0;
        double centerY = totalHeight / 2.0;

        int width = get_width();
        int height = get_height();
        if (width <= 0) {
            width = 800;
        }
        if (height <= 0) {
            height = 600;
        }

        if (totalWidth > 0 && totalHeight > 0) {
            double zoomX = static_cast<double>(width) / totalWidth * 0.9;
            double zoomY = static_cast<double>(height) / totalHeight * 0.9;
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
    m_originalIds.clear();
    m_cellSize = 0.0;
    m_gridCols = 0;
    m_panX = 0.0;
    m_panY = 0.0;
    m_zoom = 1.0;
    queue_draw();
}

void DecompositionCanvas::onDraw(const Cairo::RefPtr<Cairo::Context>& cr,
    [[maybe_unused]] int width, [[maybe_unused]] int height)
{
    // Light background
    cr->set_source_rgb(0.96, 0.96, 0.96);
    cr->paint();

    if (m_leafGraphs.empty()) {
        return;
    }

    cr->save();
    cr->translate(m_panX, m_panY);
    cr->scale(m_zoom, m_zoom);

    auto leafCount = static_cast<int>(m_leafGraphs.size());

    for (int i = 0; i < leafCount; ++i) {
        int col = i % m_gridCols;
        int row = i / m_gridCols;

        double cellLeft = static_cast<double>(col) * (m_cellSize + CELL_GAP);
        double cellTop = static_cast<double>(row) * (m_cellSize + CELL_GAP);

        auto color = getComponentColor(i);
        const auto& leaf = m_leafGraphs[static_cast<std::size_t>(i)];

        // ---- Draw cell background ----
        cr->set_source_rgb(1.0, 1.0, 1.0);
        cr->rectangle(cellLeft, cellTop, m_cellSize, m_cellSize);
        cr->fill();

        // ---- Draw cell border ----
        cr->set_source_rgb(color.r, color.g, color.b);
        cr->set_line_width(2.5 / m_zoom);
        cr->rectangle(cellLeft, cellTop, m_cellSize, m_cellSize);
        cr->stroke();

        // ---- Draw header text ----
        double headerFontSize = 11.0 / m_zoom;
        cr->set_source_rgb(0.1, 0.1, 0.1);
        cr->set_font_size(headerFontSize);

        std::string headerText = std::format("Component {} (N:{} E:{})", i + 1,
            leaf.nodeCount(), leaf.edgeCount());
        if (leaf.hasVirtualEdge()) {
            headerText += std::format(" [V:{}]", leaf.getVirtualEdges().size());
        }

        Cairo::TextExtents headerExtents;
        cr->get_text_extents(headerText, headerExtents);
        cr->move_to(cellLeft + m_cellSize / 2.0 - headerExtents.width / 2.0,
            cellTop + CELL_HEADER_HEIGHT / m_zoom * 0.75);
        cr->show_text(headerText);

        // ---- Draw header separator line ----
        double headerLineY = cellTop + CELL_HEADER_HEIGHT / m_zoom;
        cr->set_source_rgba(0.8, 0.8, 0.8, 1.0);
        cr->set_line_width(0.5 / m_zoom);
        cr->move_to(cellLeft + CELL_PADDING / m_zoom, headerLineY);
        cr->line_to(cellLeft + m_cellSize - CELL_PADDING / m_zoom, headerLineY);
        cr->stroke();

        // ---- Render abstract constraint graph inside the cell ----
        double graphAreaLeft = cellLeft + CELL_PADDING / m_zoom;
        double graphAreaTop = headerLineY + 4.0 / m_zoom;
        double graphAreaWidth = m_cellSize - 2.0 * CELL_PADDING / m_zoom;
        double graphAreaHeight = m_cellSize - CELL_HEADER_HEIGHT / m_zoom
            - CELL_PADDING / m_zoom - 4.0 / m_zoom;

        if (graphAreaWidth > 0 && graphAreaHeight > 0 && leaf.nodeCount() > 0) {
            auto bbox = computeAbstractBoundingBox(leaf);

            if (bbox.width() > 0 && bbox.height() > 0) {
                double scaleX = graphAreaWidth / bbox.width();
                double scaleY = graphAreaHeight / bbox.height();
                double scale = std::min(scaleX, scaleY) * 0.85;

                double areaCenterX = graphAreaLeft + graphAreaWidth / 2.0;
                double areaCenterY = graphAreaTop + graphAreaHeight / 2.0;

                cr->save();

                // Clip to the graph area
                cr->rectangle(graphAreaLeft, graphAreaTop, graphAreaWidth,
                    graphAreaHeight);
                cr->clip();

                // Translate to area center and scale to fit
                cr->translate(areaCenterX, areaCenterY);
                cr->scale(scale, scale);
                // Graph is centered at origin, no further translate
                // needed

                double effectiveZoom = m_zoom * scale;

                renderConstraintGraphAsGraph(cr, leaf, effectiveZoom, color.r,
                    color.g, color.b, &m_originalIds);

                cr->restore();
            }
        }
    }

    cr->restore();

    // Draw legend in top-left corner (screen space)
    if (!m_leafGraphs.empty()) {
        cr->set_font_size(12.0);
        double legendX = 10.0;
        double legendY = 20.0;

        for (int i = 0; i < leafCount; ++i) {
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
            if (leaf.hasVirtualEdge()) {
                label += std::format(" [V:{}]", leaf.getVirtualEdges().size());
            }
            cr->move_to(legendX + 20.0, legendY);
            cr->show_text(label);

            legendY += 20.0;
        }
    }
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
