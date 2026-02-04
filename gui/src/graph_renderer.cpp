#include "graph_renderer.hpp"

// General STD/STL headers
#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>

// Constraint solver headers
#include <elements.hpp>

namespace Gui {

namespace {
    constexpr double POINT_RADIUS = 5.0;
    constexpr double LINE_WIDTH = 2.0;
    constexpr double EDGE_LINE_WIDTH = 1.5;
    constexpr double VIRTUAL_EDGE_LINE_WIDTH = 2.0;
    constexpr double CONSTRAINT_FONT_SIZE = 10.0;
    constexpr double CONSTRAINT_LABEL_OFFSET = 10.0;

    constexpr std::array<ComponentColor, 8> PALETTE = { {
        { 0.20, 0.40, 0.80 }, // blue
        { 0.80, 0.30, 0.20 }, // red
        { 0.20, 0.70, 0.30 }, // green
        { 0.70, 0.40, 0.80 }, // purple
        { 0.90, 0.60, 0.10 }, // orange
        { 0.10, 0.70, 0.70 }, // teal
        { 0.80, 0.20, 0.60 }, // magenta
        { 0.50, 0.50, 0.10 }, // olive
    } };

    /**
     * @brief Get the center position of an element for edge drawing.
     * @param elem The element to query.
     * @param[out] cx Center x coordinate.
     * @param[out] cy Center y coordinate.
     */
    void getElementCenter(const Gcs::Element& elem, double& cx, double& cy)
    {
        if (elem.isElementType<Gcs::Point>()) {
            const auto& point = elem.getElement<Gcs::Point>();
            cx = point.canvasPosition.x();
            cy = point.canvasPosition.y();
        } else if (elem.isElementType<Gcs::Line>()) {
            const auto& line = elem.getElement<Gcs::Line>();
            cx = (line.canvasP1.x() + line.canvasP2.x()) / 2.0;
            cy = (line.canvasP1.y() + line.canvasP2.y()) / 2.0;
        } else if (elem.isElementType<Gcs::FixedRadiusCircle>()) {
            const auto& circle = elem.getElement<Gcs::FixedRadiusCircle>();
            cx = circle.position.x();
            cy = circle.position.y();
        }
    }
} // namespace

BoundingBox computeBoundingBox(const Gcs::ConstraintGraph& graph)
{
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();

    bool hasElements = false;

    const auto& elementMap = graph.getElementMap();
    for (const auto& node : graph.getGraph().getNodes()) {
        auto elemResult = elementMap.get(node);
        if (!elemResult.has_value())
            continue;

        const auto& elem = *elemResult.value().get();
        hasElements = true;

        if (elem.isElementType<Gcs::Point>()) {
            const auto& point = elem.getElement<Gcs::Point>();
            double px = point.canvasPosition.x();
            double py = point.canvasPosition.y();
            minX = std::min(minX, px);
            minY = std::min(minY, py);
            maxX = std::max(maxX, px);
            maxY = std::max(maxY, py);
        } else if (elem.isElementType<Gcs::Line>()) {
            const auto& line = elem.getElement<Gcs::Line>();
            minX = std::min({ minX, line.canvasP1.x(), line.canvasP2.x() });
            minY = std::min({ minY, line.canvasP1.y(), line.canvasP2.y() });
            maxX = std::max({ maxX, line.canvasP1.x(), line.canvasP2.x() });
            maxY = std::max({ maxY, line.canvasP1.y(), line.canvasP2.y() });
        } else if (elem.isElementType<Gcs::FixedRadiusCircle>()) {
            const auto& circle = elem.getElement<Gcs::FixedRadiusCircle>();
            double cx = circle.position.x();
            double cy = circle.position.y();
            double r = circle.fixedRadius;
            minX = std::min(minX, cx - r);
            minY = std::min(minY, cy - r);
            maxX = std::max(maxX, cx + r);
            maxY = std::max(maxY, cy + r);
        }
    }

    if (!hasElements) {
        return { 0.0, 0.0, 0.0, 0.0 };
    }

    // Add padding
    constexpr double PADDING = 20.0;
    return { minX - PADDING, minY - PADDING, maxX + PADDING, maxY + PADDING };
}

void renderConstraintGraph(const Cairo::RefPtr<Cairo::Context>& cr,
    const Gcs::ConstraintGraph& graph, double zoom, double componentColorR,
    double componentColorG, double componentColorB)
{
    const auto& simpleGraph = graph.getGraph();
    const auto& elementMap = graph.getElementMap();

    double pointRadius = POINT_RADIUS / zoom;
    double lineWidth = LINE_WIDTH / zoom;
    double edgeLineWidth = EDGE_LINE_WIDTH / zoom;
    double virtualEdgeLineWidth = VIRTUAL_EDGE_LINE_WIDTH / zoom;
    double fontSize = CONSTRAINT_FONT_SIZE / zoom;

    // Draw real constraint edges first (underneath)
    for (const auto& edge : simpleGraph.getEdges()) {
        if (graph.isVirtualEdge(edge))
            continue;

        auto [nodeA, nodeB] = simpleGraph.getEndpoints(edge);

        auto elemAResult = elementMap.get(nodeA);
        auto elemBResult = elementMap.get(nodeB);
        if (!elemAResult.has_value() || !elemBResult.has_value())
            continue;

        double ax = 0.0;
        double ay = 0.0;
        double bx = 0.0;
        double by = 0.0;
        getElementCenter(*elemAResult.value().get(), ax, ay);
        getElementCenter(*elemBResult.value().get(), bx, by);

        // Solid red dashed line for real constraints
        cr->set_source_rgba(0.8, 0.2, 0.2, 0.7);
        cr->set_line_width(edgeLineWidth);
        std::vector<double> dashes = { 6.0 / zoom, 4.0 / zoom };
        cr->set_dash(dashes, 0);
        cr->move_to(ax, ay);
        cr->line_to(bx, by);
        cr->stroke();
        cr->unset_dash();

        // Draw constraint label at midpoint
        auto constraint = graph.getConstraintForEdge(edge);
        if (constraint) {
            auto val = constraint->getConstraintValue();
            if (val.has_value()) {
                double mx = (ax + bx) / 2.0;
                double my = (ay + by) / 2.0;

                cr->set_source_rgb(0.8, 0.1, 0.1);
                cr->set_font_size(fontSize);

                std::string label = std::format("{:.1f}", val.value());
                Cairo::TextExtents extents;
                cr->get_text_extents(label, extents);

                cr->move_to(mx - extents.width / 2.0,
                    my - CONSTRAINT_LABEL_OFFSET / zoom);
                cr->show_text(label);
            }
        }
    }

    // Draw virtual edges (on top of real edges, underneath elements)
    for (const auto& edge : simpleGraph.getEdges()) {
        if (!graph.isVirtualEdge(edge))
            continue;

        auto [nodeA, nodeB] = simpleGraph.getEndpoints(edge);

        auto elemAResult = elementMap.get(nodeA);
        auto elemBResult = elementMap.get(nodeB);
        if (!elemAResult.has_value() || !elemBResult.has_value())
            continue;

        double ax = 0.0;
        double ay = 0.0;
        double bx = 0.0;
        double by = 0.0;
        getElementCenter(*elemAResult.value().get(), ax, ay);
        getElementCenter(*elemBResult.value().get(), bx, by);

        // Purple dashed line for virtual edges
        cr->set_source_rgba(0.55, 0.20, 0.80, 0.85);
        cr->set_line_width(virtualEdgeLineWidth);
        std::vector<double> dashes = { 8.0 / zoom, 4.0 / zoom };
        cr->set_dash(dashes, 0);
        cr->move_to(ax, ay);
        cr->line_to(bx, by);
        cr->stroke();
        cr->unset_dash();

        // Draw "V" label at midpoint to mark virtual
        double mx = (ax + bx) / 2.0;
        double my = (ay + by) / 2.0;

        cr->set_source_rgba(0.55, 0.20, 0.80, 1.0);
        cr->set_font_size(fontSize);

        Cairo::TextExtents extents;
        cr->get_text_extents("V", extents);
        cr->move_to(
            mx - extents.width / 2.0, my - CONSTRAINT_LABEL_OFFSET / zoom);
        cr->show_text("V");
    }

    // Draw elements on top
    for (const auto& node : simpleGraph.getNodes()) {
        auto elemResult = elementMap.get(node);
        if (!elemResult.has_value())
            continue;

        const auto& elem = *elemResult.value().get();

        if (elem.isElementType<Gcs::Point>()) {
            const auto& point = elem.getElement<Gcs::Point>();
            double px = point.canvasPosition.x();
            double py = point.canvasPosition.y();

            // Filled circle
            cr->set_source_rgb(
                componentColorR, componentColorG, componentColorB);
            cr->arc(px, py, pointRadius, 0, 2.0 * std::numbers::pi);
            cr->fill();

            // Outline
            cr->set_source_rgb(0.0, 0.0, 0.0);
            cr->set_line_width(1.0 / zoom);
            cr->arc(px, py, pointRadius, 0, 2.0 * std::numbers::pi);
            cr->stroke();

        } else if (elem.isElementType<Gcs::Line>()) {
            const auto& line = elem.getElement<Gcs::Line>();

            cr->set_source_rgb(
                componentColorR, componentColorG, componentColorB);
            cr->set_line_width(lineWidth);
            cr->move_to(line.canvasP1.x(), line.canvasP1.y());
            cr->line_to(line.canvasP2.x(), line.canvasP2.y());
            cr->stroke();

            // Endpoint dots
            double epRadius = pointRadius * 0.6;
            cr->set_source_rgb(componentColorR * 0.8, componentColorG * 0.8,
                componentColorB * 0.8);
            cr->arc(line.canvasP1.x(), line.canvasP1.y(), epRadius, 0,
                2.0 * std::numbers::pi);
            cr->fill();
            cr->arc(line.canvasP2.x(), line.canvasP2.y(), epRadius, 0,
                2.0 * std::numbers::pi);
            cr->fill();

        } else if (elem.isElementType<Gcs::FixedRadiusCircle>()) {
            const auto& circle = elem.getElement<Gcs::FixedRadiusCircle>();

            cr->set_source_rgb(
                componentColorR, componentColorG, componentColorB);
            cr->set_line_width(lineWidth);
            cr->arc(circle.position.x(), circle.position.y(),
                circle.fixedRadius, 0, 2.0 * std::numbers::pi);
            cr->stroke();

            // Center dot
            cr->set_source_rgb(componentColorR * 0.8, componentColorG * 0.8,
                componentColorB * 0.8);
            cr->arc(circle.position.x(), circle.position.y(), pointRadius * 0.5,
                0, 2.0 * std::numbers::pi);
            cr->fill();
        }
    }
}

ComponentColor getComponentColor(int index)
{
    return PALETTE[static_cast<std::size_t>(index) % PALETTE.size()];
}

} // namespace Gui
