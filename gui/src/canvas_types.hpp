#ifndef GUI_CANVAS_TYPES_HPP
#define GUI_CANVAS_TYPES_HPP

// General STD/STL headers
#include <cstdint>

namespace Gui {

using ElementId = uint32_t;
using ConstraintId = uint32_t;

/// @brief Available editing tools in the modeller.
enum class Tool {
    Select, ///< Select and drag elements.
    Point, ///< Place a point on click.
    Line, ///< Place a line (two clicks for endpoints).
    DistanceConstraint, ///< Add distance constraint between two elements.
    Delete, ///< Delete an element or constraint on click.
};

/// @brief Type tag for visual canvas elements.
enum class CanvasElementType {
    Point,
    Line,
};

/// @brief A lightweight record of a visual element on the canvas.
struct CanvasElement {
    ElementId id;
    CanvasElementType type;
    double x; ///< Position x (or first endpoint x for lines).
    double y; ///< Position y (or first endpoint y for lines).
    double x2; ///< Second endpoint x (lines only).
    double y2; ///< Second endpoint y (lines only).
    bool selected = false;
};

/// @brief A lightweight record of a visual constraint on the canvas.
struct CanvasConstraint {
    ConstraintId id;
    ElementId elementA;
    ElementId elementB;
    double value;
};

} // namespace Gui

#endif // GUI_CANVAS_TYPES_HPP
