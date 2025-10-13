#include "elements.hpp"
#include <format>

namespace Gcs {

// Point implementation
Point::Point()
    : OiriginalXOnCanvas { 0.0 }
    , OriginalYOnCanvas { 0.0 }
    , x { 0.0 }
    , y { 0.0 }
{
}

Point::Point(double xCoordCanvas, double yCoordCanvas)
    : OiriginalXOnCanvas { xCoordCanvas }
    , OriginalYOnCanvas { yCoordCanvas }
    , x { 0.0 }
    , y { 0.0 }
{
}

std::string Point::getTypeName() const { return "Point"; }

std::string Point::toString() const
{
    return std::format("CanvasCoords({},{}), Calculated({},{})",
        OiriginalXOnCanvas, OriginalYOnCanvas, x, y);
}

void Point::updateElementPosition(double new_x, double new_y)
{
    x = new_x;
    y = new_y;
}

// FixedRadiusCircle implementation
FixedRadiusCircle::FixedRadiusCircle()
    : x { 0.0 }
    , y { 0.0 }
    , fixed_radius { 0.0 }
{
}

FixedRadiusCircle::FixedRadiusCircle(double x_coord, double y_coord, double r)
    : x { x_coord }
    , y { y_coord }
    , fixed_radius { r }
{
}

std::string FixedRadiusCircle::getTypeName() const
{
    return "FixedRadiusCircle";
}

std::string FixedRadiusCircle::toString() const
{
    return std::format(
        "FixedRadiusCircle(x: {}, y: {}, radius: {})", x, y, fixed_radius);
}

void FixedRadiusCircle::updateElementPosition(double new_x, double new_y)
{
    x = new_x;
    y = new_y;
}

// Line implementation
Line::Line()
    : originalX1OnCanvas { 0.0 }
    , originalY1OnCanvas { 0.0 }
    , originalX2OnCanvas { 0.0 }
    , originalY2OnCanvas { 0.0 }
    , x1 { 0.0 }
    , y1 { 0.0 }
    , x2 { 0.0 }
    , y2 { 0.0 }
{
}

Line::Line(double x1Canvas, double y1Canvas, double x2Canvas, double y2Canvas)
    : originalX1OnCanvas { x1Canvas }
    , originalY1OnCanvas { y1Canvas }
    , originalX2OnCanvas { x2Canvas }
    , originalY2OnCanvas { y2Canvas }
    , x1 { 0.0 }
    , y1 { 0.0 }
    , x2 { 0.0 }
    , y2 { 0.0 }
{
}

std::string Line::getTypeName() const { return "Line"; }

std::string Line::toString() const
{
    return std::format(
        "Line(CanvasP1:({},{}), CanvasP2:({},{}), CalcP1:({},{}), CalcP2:({},{}))",
        originalX1OnCanvas, originalY1OnCanvas,
        originalX2OnCanvas, originalY2OnCanvas,
        x1, y1, x2, y2);
}

void Line::updateElementPosition(double new_x1, double new_y1, double new_x2, double new_y2)
{
    this->x1 = new_x1;
    this->y1 = new_y1;
    this->x2 = new_x2;
    this->y2 = new_y2;
}

// Element class implementation
std::string Element::getElementName() const
{
    return std::visit(
        [](const auto& element) { return element.getTypeName(); }, element);
}

std::string Element::toString() const
{
    return std::visit(
        [](const auto& element) { return element.toString(); }, element);
}

} // namespace Gcs
