#include "stree_canvas.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

namespace Gui {

namespace {
    constexpr double ZOOM_FACTOR = 1.1;
    constexpr double MIN_ZOOM = 0.05;
    constexpr double MAX_ZOOM = 20.0;

    // Tree layout constants (in world coordinates)
    constexpr double NODE_WIDTH = 220.0;
    constexpr double NODE_HEIGHT = 180.0;
    constexpr double HORIZONTAL_GAP = 30.0;
    constexpr double VERTICAL_GAP = 60.0;
    constexpr double NODE_PADDING = 10.0;

    // Header area inside each node box for info text
    constexpr double HEADER_HEIGHT = 20.0;

    constexpr double LEAF_BORDER_WIDTH = 3.0;
    constexpr double INTERNAL_BORDER_WIDTH = 2.0;
    constexpr double TREE_EDGE_WIDTH = 2.0;
} // namespace

STreeCanvas::STreeCanvas()
{
    set_hexpand(true);
    set_vexpand(true);

    set_draw_func(sigc::mem_fun(*this, &STreeCanvas::onDraw));

    // Drag gesture for panning
    m_dragGesture = Gtk::GestureDrag::create();
    m_dragGesture->set_button(GDK_BUTTON_PRIMARY);
    m_dragGesture->signal_drag_begin().connect(
        sigc::mem_fun(*this, &STreeCanvas::onDragBegin));
    m_dragGesture->signal_drag_update().connect(
        sigc::mem_fun(*this, &STreeCanvas::onDragUpdate));
    m_dragGesture->signal_drag_end().connect(
        sigc::mem_fun(*this, &STreeCanvas::onDragEnd));
    add_controller(m_dragGesture);

    // Scroll controller for zoom
    m_scrollController = Gtk::EventControllerScroll::create();
    m_scrollController->set_flags(Gtk::EventControllerScroll::Flags::VERTICAL);
    m_scrollController->signal_scroll().connect(
        sigc::mem_fun(*this, &STreeCanvas::onScroll), false);
    add_controller(m_scrollController);

    // Motion controller
    m_motionController = Gtk::EventControllerMotion::create();
    m_motionController->signal_motion().connect(
        sigc::mem_fun(*this, &STreeCanvas::onMotion));
    add_controller(m_motionController);
}

void STreeCanvas::setDecomposition(
    const MathUtils::BinaryTree<Gcs::ConstraintGraph>& stree)
{
    m_stree = stree;

    // Build original-ID map from the root graph.
    auto root = m_stree.getRoot();
    if (root.has_value()) {
        m_originalIds = buildOriginalIdMap(m_stree.getValue(*root));
    } else {
        m_originalIds.clear();
    }

    computeLayout();

    // Center the view on the tree layout
    if (!m_layout.empty()) {
        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double maxY = std::numeric_limits<double>::lowest();

        for (const auto& [id, layout] : m_layout) {
            minX = std::min(minX, layout.x - layout.width / 2.0);
            minY = std::min(minY, layout.y - layout.height / 2.0);
            maxX = std::max(maxX, layout.x + layout.width / 2.0);
            maxY = std::max(maxY, layout.y + layout.height / 2.0);
        }

        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;

        int width = get_width();
        int height = get_height();
        if (width <= 0)
            width = 800;
        if (height <= 0)
            height = 600;

        double rangeX = maxX - minX;
        double rangeY = maxY - minY;
        if (rangeX > 0 && rangeY > 0) {
            double zoomX = static_cast<double>(width) / rangeX * 0.85;
            double zoomY = static_cast<double>(height) / rangeY * 0.85;
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

void STreeCanvas::clear()
{
    m_stree = {};
    m_layout.clear();
    m_originalIds.clear();
    m_panX = 0.0;
    m_panY = 0.0;
    m_zoom = 1.0;
    queue_draw();
}

void STreeCanvas::computeLayout()
{
    m_layout.clear();
    auto root = m_stree.getRoot();
    if (!root.has_value())
        return;

    int leafCounter = 0;
    layoutSubtree(*root, 0.0, 0.0, leafCounter);
}

double STreeCanvas::layoutSubtree(MathUtils::TreeNodeId nodeId,
    double leftBound, double depth, int& leafCounter)
{
    bool isLeaf = m_stree.isLeaf(nodeId);

    double y = depth * (NODE_HEIGHT + VERTICAL_GAP);

    if (isLeaf) {
        double x = leftBound + NODE_WIDTH / 2.0;
        m_layout[nodeId] = NodeLayout {
            .x = x,
            .y = y,
            .width = NODE_WIDTH,
            .height = NODE_HEIGHT,
            .isLeaf = true,
            .colorIndex = leafCounter++,
        };
        return leftBound + NODE_WIDTH + HORIZONTAL_GAP;
    }

    // Layout children first to determine this node's position
    double childRightBound = leftBound;

    auto leftChild = m_stree.getLeftChild(nodeId);
    double leftChildCenter = 0.0;
    if (leftChild.has_value()) {
        childRightBound = layoutSubtree(
            *leftChild, childRightBound, depth + 1, leafCounter);
        leftChildCenter = m_layout[*leftChild].x;
    }

    auto rightChild = m_stree.getRightChild(nodeId);
    double rightChildCenter = 0.0;
    if (rightChild.has_value()) {
        childRightBound = layoutSubtree(
            *rightChild, childRightBound, depth + 1, leafCounter);
        rightChildCenter = m_layout[*rightChild].x;
    }

    // Position parent centered above its children
    double x = leftBound + NODE_WIDTH / 2.0;
    if (leftChild.has_value() && rightChild.has_value()) {
        x = (leftChildCenter + rightChildCenter) / 2.0;
    } else if (leftChild.has_value()) {
        x = leftChildCenter;
    } else if (rightChild.has_value()) {
        x = rightChildCenter;
    }

    // Ensure parent node doesn't overlap with left bound
    x = std::max(x, leftBound + NODE_WIDTH / 2.0);

    m_layout[nodeId] = NodeLayout {
        .x = x,
        .y = y,
        .width = NODE_WIDTH,
        .height = NODE_HEIGHT,
        .isLeaf = false,
        .colorIndex = 0,
    };

    return std::max(childRightBound, x + NODE_WIDTH / 2.0 + HORIZONTAL_GAP);
}

void STreeCanvas::onDraw(const Cairo::RefPtr<Cairo::Context>& cr,
    [[maybe_unused]] int width, [[maybe_unused]] int height)
{
    // Light gray background
    cr->set_source_rgb(0.96, 0.96, 0.96);
    cr->paint();

    if (m_stree.isEmpty())
        return;

    cr->save();
    cr->translate(m_panX, m_panY);
    cr->scale(m_zoom, m_zoom);

    drawTreeEdges(cr);
    drawTreeNodes(cr);

    cr->restore();
}

void STreeCanvas::drawTreeEdges(const Cairo::RefPtr<Cairo::Context>& cr)
{
    cr->set_source_rgba(0.4, 0.4, 0.4, 0.8);
    cr->set_line_width(TREE_EDGE_WIDTH / m_zoom);

    for (const auto& [nodeId, layout] : m_layout) {
        auto leftChild = m_stree.getLeftChild(nodeId);
        if (leftChild.has_value()) {
            auto childIt = m_layout.find(*leftChild);
            if (childIt != m_layout.end()) {
                double parentBottomX = layout.x;
                double parentBottomY = layout.y + layout.height / 2.0;
                double childTopX = childIt->second.x;
                double childTopY
                    = childIt->second.y - childIt->second.height / 2.0;

                cr->move_to(parentBottomX, parentBottomY);
                cr->line_to(childTopX, childTopY);
                cr->stroke();

                // Draw "L" label near the connection point
                cr->set_font_size(12.0 / m_zoom);
                double labelX
                    = (parentBottomX + childTopX) / 2.0 - 10.0 / m_zoom;
                double labelY = (parentBottomY + childTopY) / 2.0;
                cr->set_source_rgba(0.2, 0.5, 0.2, 0.9);
                cr->move_to(labelX, labelY);
                cr->show_text("L");
                cr->set_source_rgba(0.4, 0.4, 0.4, 0.8);
            }
        }

        auto rightChild = m_stree.getRightChild(nodeId);
        if (rightChild.has_value()) {
            auto childIt = m_layout.find(*rightChild);
            if (childIt != m_layout.end()) {
                double parentBottomX = layout.x;
                double parentBottomY = layout.y + layout.height / 2.0;
                double childTopX = childIt->second.x;
                double childTopY
                    = childIt->second.y - childIt->second.height / 2.0;

                cr->move_to(parentBottomX, parentBottomY);
                cr->line_to(childTopX, childTopY);
                cr->stroke();

                // Draw "R" label near the connection point
                cr->set_font_size(12.0 / m_zoom);
                double labelX
                    = (parentBottomX + childTopX) / 2.0 + 4.0 / m_zoom;
                double labelY = (parentBottomY + childTopY) / 2.0;
                cr->set_source_rgba(0.2, 0.2, 0.7, 0.9);
                cr->move_to(labelX, labelY);
                cr->show_text("R");
                cr->set_source_rgba(0.4, 0.4, 0.4, 0.8);
            }
        }
    }
}

void STreeCanvas::drawTreeNodes(const Cairo::RefPtr<Cairo::Context>& cr)
{
    for (const auto& [nodeId, layout] : m_layout) {
        double left = layout.x - layout.width / 2.0;
        double top = layout.y - layout.height / 2.0;

        // Draw background rectangle
        cr->set_source_rgb(1.0, 1.0, 1.0);
        cr->rectangle(left, top, layout.width, layout.height);
        cr->fill();

        // Draw border
        if (layout.isLeaf) {
            auto color = getComponentColor(layout.colorIndex);
            cr->set_source_rgb(color.r, color.g, color.b);
            cr->set_line_width(LEAF_BORDER_WIDTH / m_zoom);
        } else {
            cr->set_source_rgb(0.5, 0.5, 0.5);
            cr->set_line_width(INTERNAL_BORDER_WIDTH / m_zoom);
        }
        cr->rectangle(left, top, layout.width, layout.height);
        cr->stroke();

        // Draw header info
        const auto& graph = m_stree.getValue(nodeId);
        cr->set_source_rgb(0.1, 0.1, 0.1);
        cr->set_font_size(10.0 / m_zoom);

        std::string headerText
            = std::format("N:{} E:{}", graph.nodeCount(), graph.edgeCount());
        if (graph.hasVirtualEdge()) {
            headerText += std::format(" V:{}", graph.getVirtualEdges().size());
        }
        if (layout.isLeaf) {
            headerText += " [leaf]";
        }

        Cairo::TextExtents extents;
        cr->get_text_extents(headerText, extents);
        cr->move_to(
            layout.x - extents.width / 2.0, top + HEADER_HEIGHT / m_zoom * 0.8);
        cr->show_text(headerText);

        // Draw separator line below header
        cr->set_source_rgba(0.8, 0.8, 0.8, 1.0);
        cr->set_line_width(0.5 / m_zoom);
        double headerLineY = top + HEADER_HEIGHT / m_zoom;
        cr->move_to(left + NODE_PADDING / m_zoom, headerLineY);
        cr->line_to(left + layout.width - NODE_PADDING / m_zoom, headerLineY);
        cr->stroke();

        // Render the miniature abstract graph inside the node box.
        // The graph area is below the header.
        double graphAreaLeft = left + NODE_PADDING / m_zoom;
        double graphAreaTop = headerLineY + 2.0 / m_zoom;
        double graphAreaWidth = layout.width - 2.0 * NODE_PADDING / m_zoom;
        double graphAreaHeight = layout.height - HEADER_HEIGHT / m_zoom
            - 2.0 * NODE_PADDING / m_zoom;

        if (graphAreaWidth > 0 && graphAreaHeight > 0
            && graph.nodeCount() > 0) {
            auto bbox = computeAbstractBoundingBox(graph);

            if (bbox.width() > 0 && bbox.height() > 0) {
                // Compute scale to fit the abstract layout into the
                // node box
                double scaleX = graphAreaWidth / bbox.width();
                double scaleY = graphAreaHeight / bbox.height();
                double scale = std::min(scaleX, scaleY) * 0.85;

                // The abstract layout is centered at origin, so we
                // just translate to the area center and scale.
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

                // The effective zoom for line width calculations
                double effectiveZoom = m_zoom * scale;

                if (layout.isLeaf) {
                    auto color = getComponentColor(layout.colorIndex);
                    renderConstraintGraphAsGraph(cr, graph, effectiveZoom,
                        color.r, color.g, color.b, &m_originalIds);
                } else {
                    renderConstraintGraphAsGraph(cr, graph, effectiveZoom, 0.3,
                        0.3, 0.3, &m_originalIds);
                }

                cr->restore();
            }
        }
    }
}

void STreeCanvas::screenToWorld(
    double sx, double sy, double& wx, double& wy) const
{
    wx = (sx - m_panX) / m_zoom;
    wy = (sy - m_panY) / m_zoom;
}

void STreeCanvas::onDragBegin(double startX, double startY)
{
    m_dragStartX = startX;
    m_dragStartY = startY;
    m_dragOrigPanX = m_panX;
    m_dragOrigPanY = m_panY;
}

void STreeCanvas::onDragUpdate(double offsetX, double offsetY)
{
    m_panX = m_dragOrigPanX + offsetX;
    m_panY = m_dragOrigPanY + offsetY;
    queue_draw();
}

void STreeCanvas::onDragEnd(
    [[maybe_unused]] double offsetX, [[maybe_unused]] double offsetY)
{
    // Nothing needed
}

bool STreeCanvas::onScroll([[maybe_unused]] double dx, double dy)
{
    double factor = (dy < 0) ? ZOOM_FACTOR : (1.0 / ZOOM_FACTOR);
    double newZoom = m_zoom * factor;
    newZoom = std::clamp(newZoom, MIN_ZOOM, MAX_ZOOM);

    double mouseWorldX = (m_mouseScreenX - m_panX) / m_zoom;
    double mouseWorldY = (m_mouseScreenY - m_panY) / m_zoom;

    m_zoom = newZoom;
    m_panX = m_mouseScreenX - mouseWorldX * m_zoom;
    m_panY = m_mouseScreenY - mouseWorldY * m_zoom;

    queue_draw();
    return true;
}

void STreeCanvas::onMotion(double x, double y)
{
    m_mouseScreenX = x;
    m_mouseScreenY = y;
}

} // namespace Gui
