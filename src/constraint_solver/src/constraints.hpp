#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

// General STD/STL headers
#include <concepts>
#include <expected>
#include <memory>
#include <string>
#include <variant>

// Thirdparty headers
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace Gcs {

inline std::shared_ptr<spdlog::logger> getConstraintLogger()
{
    static auto s_gcsLogger = spdlog::stdout_color_mt("CONSTRAINT_LOGGER");
    return s_gcsLogger;
}

/// @brief Errors from constraint value access operations.
enum class ConstraintError {
    NoValue, ///< Constraint type does not carry a numeric value.
};

template <typename T>
concept ConstraintType = requires(T constraint) {
    { constraint.getTypeName() } -> std::convertible_to<std::string>;
    {
        constraint.getConstraintValue()
    } -> std::same_as<std::expected<double, ConstraintError>>;
};

struct DistanceConstraint {
    double distance;

    explicit DistanceConstraint(double d);

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

struct TangencyConstraint {
    double angle;

    explicit TangencyConstraint(double d);

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

struct PointOnLineConstraint {
    explicit PointOnLineConstraint();

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

struct VirtualConstraint {
    explicit VirtualConstraint();

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

template <ConstraintType... Types>
using ConstraintInterface = std::variant<Types...>;

using ConstraintVariant = ConstraintInterface<DistanceConstraint,
    TangencyConstraint, PointOnLineConstraint, VirtualConstraint>;

class Constraint final {
private:
    ConstraintVariant m_constraint;

public:
    template <typename T>
    explicit Constraint(const T& c)
        : m_constraint { c }
    {
    }

    template <typename T>
    bool isConstraintType() const
    {
        return std::holds_alternative<T>(m_constraint);
    }

    std::string getConstraintName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

} // namespace Gcs

#endif // CONSTRAINTS_HPP
