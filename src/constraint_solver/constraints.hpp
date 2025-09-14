#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include <string>
#include <variant>

namespace Gcs {

template <typename T>
concept ConstraintType = requires(T constraint) {
    { constraint.getTypeName() } -> std::convertible_to<std::string>;
    { constraint.getConstraintValue() } -> std::convertible_to<double>;
};

struct DistanceConstraint {
    float distance;
    explicit DistanceConstraint(float d)
        : distance { d }
    {
    }
    std::string getTypeName() const { return "Distance"; }
    double getConstraintValue() const { return distance; }
};
struct TangencyConstraint {
    float angle;
    explicit TangencyConstraint(float d)
        : angle { d }
    {
    }

    std::string getTypeName() const { return "Tangency"; }
    double getConstraintValue() const { return angle; }
};

template <ConstraintType... Types>
using ConstraintInterface = std::variant<Types...>;

class Constraint final {
private:
    ConstraintInterface<DistanceConstraint, TangencyConstraint> constraint;

public:
    template <typename T>
    explicit Constraint()
        : constraint { T {} }
    {
    }

    template <typename T>
    explicit Constraint(const T& c)
        : constraint { c }
    {
    }

    template <typename T> bool isConstraintType() const
    {
        return std::holds_alternative<T>(constraint);
    }

    template <typename Visitor> auto visitConsraint(Visitor&& visitor)
    {
        return std::visit(visitor, constraint);
    }

    std::string getConstraintName() const
    {
        return std::visit(
            [](const auto& constraint) { return constraint.getTypeName(); },
            constraint);
    }

    double getConstraintValue()
    {
        return std::visit(
            [](const auto& constraint) {
                return constraint.getConstraintValue();
            },
            constraint);
    }
};

} // namespace Gcs

#endif // CONSTRAINTS_HPP
