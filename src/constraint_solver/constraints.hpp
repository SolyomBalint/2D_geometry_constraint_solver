#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include <memory>
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <string>
#include <variant>

namespace Gcs {

inline std::shared_ptr<spdlog::logger> getConstraintLogger()
{
    static auto gcsLogger = spdlog::stdout_color_mt("CONSTRAINT_LOGGER");
    return gcsLogger;
}

template <typename T>
concept ConstraintType = requires(T constraint) {
    { constraint.getTypeName() } -> std::convertible_to<std::string>;
    { constraint.getConstraintValue() } -> std::convertible_to<double>;
};

struct DistanceConstraint {
    double distance;

    explicit DistanceConstraint(double d);

    std::string getTypeName() const;
    double getConstraintValue() const;
};

struct TangencyConstraint {
    double angle;

    explicit TangencyConstraint(double d);

    std::string getTypeName() const;
    double getConstraintValue() const;
};

struct PointOnLineConstraint {
    explicit PointOnLineConstraint();

    std::string getTypeName() const;
    double getConstraintValue() const;
};

struct VirtualConstraint {
    explicit VirtualConstraint();

    std::string getTypeName() const;
    double getConstraintValue() const;
};

template <ConstraintType... Types>
using ConstraintInterface = std::variant<Types...>;

class Constraint final {
private:
    ConstraintInterface<DistanceConstraint, TangencyConstraint,
        PointOnLineConstraint, VirtualConstraint>
        constraint;

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

    std::string getConstraintName() const;
    double getConstraintValue();
};

} // namespace Gcs

#endif // CONSTRAINTS_HPP
