#include "constraints.hpp"
#include <cfloat>

namespace Gcs {

// DistanceConstraint implementation
DistanceConstraint::DistanceConstraint(double d)
    : distance { d }
{
}

std::string DistanceConstraint::getTypeName() const { return "Distance"; }

double DistanceConstraint::getConstraintValue() const { return distance; }

// TangencyConstraint implementation
TangencyConstraint::TangencyConstraint(double d)
    : angle { d }
{
}

std::string TangencyConstraint::getTypeName() const { return "Tangency"; }

double TangencyConstraint::getConstraintValue() const { return angle; }

PointOnLineConstraint::PointOnLineConstraint() { }

std::string PointOnLineConstraint::getTypeName() const { return "PointOnLine"; }

// TODO document what this means
double PointOnLineConstraint::getConstraintValue() const { return DBL_MAX; }

// VirtualConstraint implementation
VirtualConstraint::VirtualConstraint() { }

std::string VirtualConstraint::getTypeName() const { return "Virtual"; }

double VirtualConstraint::getConstraintValue() const
{
    getConstraintLogger()->error(
        "The value of a virtual constraint is querried");
    return -1.0;
}

// Constraint class implementation
std::string Constraint::getConstraintName() const
{
    return std::visit(
        [](const auto& constraint) { return constraint.getTypeName(); },
        constraint);
}

double Constraint::getConstraintValue()
{
    return std::visit(
        [](const auto& constraint) { return constraint.getConstraintValue(); },
        constraint);
}

} // namespace Gcs
