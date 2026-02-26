#include "constraints.hpp"

namespace Gcs {

// DistanceConstraint implementation
DistanceConstraint::DistanceConstraint(double d)
    : distance { d }
{
}

std::string DistanceConstraint::getTypeName() const
{
    return "Distance";
}

std::expected<double, ConstraintError>
DistanceConstraint::getConstraintValue() const
{
    return distance;
}

// TangencyConstraint implementation
TangencyConstraint::TangencyConstraint(double d)
    : angle { d }
{
}

std::string TangencyConstraint::getTypeName() const
{
    return "Tangency";
}

std::expected<double, ConstraintError>
TangencyConstraint::getConstraintValue() const
{
    return angle;
}

// AngleConstraint implementation
AngleConstraint::AngleConstraint(double a)
    : angle { a }
{
}

std::string AngleConstraint::getTypeName() const
{
    return "Angle";
}

std::expected<double, ConstraintError>
AngleConstraint::getConstraintValue() const
{
    return angle;
}

PointOnLineConstraint::PointOnLineConstraint() { }

std::string PointOnLineConstraint::getTypeName() const
{
    return "PointOnLine";
}

std::expected<double, ConstraintError>
PointOnLineConstraint::getConstraintValue() const
{
    return std::unexpected(ConstraintError::NoValue);
}

// VirtualConstraint implementation
VirtualConstraint::VirtualConstraint() { }

std::string VirtualConstraint::getTypeName() const
{
    return "Virtual";
}

std::expected<double, ConstraintError>
VirtualConstraint::getConstraintValue() const
{
    return std::unexpected(ConstraintError::NoValue);
}

// Constraint class implementation
std::string Constraint::getConstraintName() const
{
    return std::visit(
        [](const auto& constr) { return constr.getTypeName(); }, m_constraint);
}

std::expected<double, ConstraintError> Constraint::getConstraintValue() const
{
    return std::visit(
        [](const auto& constr) { return constr.getConstraintValue(); },
        m_constraint);
}

} // namespace Gcs
