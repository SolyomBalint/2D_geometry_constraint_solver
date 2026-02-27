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
    AngleConstraint, ///< Add angle constraint between two lines.
    Delete, ///< Delete an element or constraint on click.
};

/// @brief Type tag for visual canvas elements.
enum class CanvasElementType {
    Point,
    Line,
};

/// @brief Type tag for visual canvas constraints.
enum class CanvasConstraintType {
    Distance, ///< Distance constraint (shown as dashed line + value).
    Angle, ///< Angle constraint between two lines (shown as arc + degrees).
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
    CanvasConstraintType type = CanvasConstraintType::Distance;
    /// For angle constraints: when true, the arc is drawn on the
    /// opposite (supplementary) side of the two lines.
    bool flipped = false;
};

} // namespace Gui

#endif // GUI_CANVAS_TYPES_HPP
