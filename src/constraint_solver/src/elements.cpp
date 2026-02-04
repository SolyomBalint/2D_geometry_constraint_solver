#include "elements.hpp"
#include <format>

namespace Gcs {

// Point implementation
Point::Point()
    : canvasPosition { Eigen::Vector2d::Zero() }
    , position { Eigen::Vector2d::Zero() }
{
}

Point::Point(const Eigen::Vector2d& canvasPos)
    : canvasPosition { canvasPos }
    , position { Eigen::Vector2d::Zero() }
{
}

std::string Point::getTypeName() const
{
    return "Point";
}

std::string Point::toString() const
{
    return std::format("CanvasCoords({},{}), Calculated({},{})",
        canvasPosition.x(), canvasPosition.y(), position.x(), position.y());
}

void Point::updateElementPosition(const Eigen::Vector2d& newPosition)
{
    position = newPosition;
}

// FixedRadiusCircle implementation
FixedRadiusCircle::FixedRadiusCircle()
    : position { Eigen::Vector2d::Zero() }
    , fixedRadius { 0.0 }
{
}

FixedRadiusCircle::FixedRadiusCircle(const Eigen::Vector2d& centerPos, double r)
    : position { centerPos }
    , fixedRadius { r }
{
}

std::string FixedRadiusCircle::getTypeName() const
{
    return "FixedRadiusCircle";
}

std::string FixedRadiusCircle::toString() const
{
    return std::format("FixedRadiusCircle(x: {}, y: {}, radius: {})",
        position.x(), position.y(), fixedRadius);
}

void FixedRadiusCircle::updateElementPosition(
    const Eigen::Vector2d& newPosition)
{
    position = newPosition;
}

// Line implementation
Line::Line()
    : canvasP1 { Eigen::Vector2d::Zero() }
    , canvasP2 { Eigen::Vector2d::Zero() }
    , p1 { Eigen::Vector2d::Zero() }
    , p2 { Eigen::Vector2d::Zero() }
{
}

Line::Line(const Eigen::Vector2d& canvasEndpoint1,
    const Eigen::Vector2d& canvasEndpoint2)
    : canvasP1 { canvasEndpoint1 }
    , canvasP2 { canvasEndpoint2 }
    , p1 { Eigen::Vector2d::Zero() }
    , p2 { Eigen::Vector2d::Zero() }
{
}

std::string Line::getTypeName() const
{
    return "Line";
}

std::string Line::toString() const
{
    return std::format("Line(CanvasP1:({},{}), CanvasP2:({},{}), "
                       "CalcP1:({},{}), CalcP2:({},{}))",
        canvasP1.x(), canvasP1.y(), canvasP2.x(), canvasP2.y(), p1.x(), p1.y(),
        p2.x(), p2.y());
}

void Line::updateElementPosition(
    const Eigen::Vector2d& newP1, const Eigen::Vector2d& newP2)
{
    p1 = newP1;
    p2 = newP2;
}

Eigen::Vector2d Line::direction() const
{
    return p2 - p1;
}

Eigen::Vector2d Line::unitDirection() const
{
    return (p2 - p1).normalized();
}

Eigen::Vector2d Line::normal() const
{
    Eigen::Vector2d dir = direction();
    return Eigen::Vector2d(-dir.y(), dir.x());
}

double Line::length() const
{
    return (p2 - p1).norm();
}

Eigen::Vector2d Line::midpoint() const
{
    return (p1 + p2) / 2.0;
}

// Element class implementation
std::string Element::getElementName() const
{
    return std::visit(
        [](const auto& elem) { return elem.getTypeName(); }, m_element);
}

std::string Element::toString() const
{
    return std::visit(
        [](const auto& elem) { return elem.toString(); }, m_element);
}

} // namespace Gcs
