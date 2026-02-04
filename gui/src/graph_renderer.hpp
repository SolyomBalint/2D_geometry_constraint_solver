#ifndef GUI_GRAPH_RENDERER_HPP
#define GUI_GRAPH_RENDERER_HPP

// General STD/STL headers
#include <vector>

// Constraint solver headers
#include <gcs_data_structures.hpp>

// Thirdparty headers
#include <cairomm/context.h>

namespace Gui {

/**
 * @brief Axis-aligned bounding box for a set of elements.
 */
struct BoundingBox {
    double minX = 0.0;
    double minY = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;

    double width() const { return maxX - minX; }
    double height() const { return maxY - minY; }
    double centerX() const { return (minX + maxX) / 2.0; }
    double centerY() const { return (minY + maxY) / 2.0; }
};

/**
 * @brief Compute the bounding box of all elements in a constraint graph.
 * @param graph The constraint graph whose elements to measure.
 * @return The axis-aligned bounding box. If the graph has no elements
 *         with positions, returns a zero-size box at the origin.
 */
BoundingBox computeBoundingBox(const Gcs::ConstraintGraph& graph);

/**
 * @brief Render a constraint graph using Cairo.
 *
 * Draws all elements (points as circles, lines as segments) and all
 * edges (real constraints as solid red dashed lines, virtual edges as
 * purple dashed lines). The caller is responsible for setting up the
 * Cairo coordinate transform (translation, scaling) before calling
 * this function.
 *
 * @param cr The Cairo context to draw into.
 * @param graph The constraint graph to render.
 * @param zoom The current zoom level, used to keep visual sizes
 *             (point radius, line width) constant on screen.
 * @param componentColorR Red component [0,1] for the component tint.
 * @param componentColorG Green component [0,1] for the component tint.
 * @param componentColorB Blue component [0,1] for the component tint.
 */
void renderConstraintGraph(const Cairo::RefPtr<Cairo::Context>& cr,
    const Gcs::ConstraintGraph& graph, double zoom,
    double componentColorR = 0.2, double componentColorG = 0.2,
    double componentColorB = 0.2);

/**
 * @brief A set of distinct colors for distinguishing components.
 *
 * Each entry is an {R, G, B} triplet with values in [0, 1].
 */
struct ComponentColor {
    double r;
    double g;
    double b;
};

/**
 * @brief Get a color from a palette, cycling if index exceeds palette size.
 * @param index The component index.
 * @return A distinct color.
 */
ComponentColor getComponentColor(int index);

} // namespace Gui

#endif // GUI_GRAPH_RENDERER_HPP
