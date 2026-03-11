#ifndef GCS_MODEL_CONSTRAINTS_HPP
#define GCS_MODEL_CONSTRAINTS_HPP

// General STD/STL headers
#include <concepts>
#include <expected>
#include <memory>
#include <string>
#include <variant>

// Custom headers
#include <gcs/export.hpp>

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

struct GCS_API DistanceConstraint {
    double distance;

    explicit DistanceConstraint(double d);

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

struct GCS_API TangencyConstraint {
    double angle;

    explicit TangencyConstraint(double d);

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

struct GCS_API AngleConstraint {
    double angle;
    /// When true, the solver flips the orientation heuristic so the
    /// angle is enforced on the opposite side of the two lines.
    bool flipOrientation = false;

    explicit AngleConstraint(double a, bool flip = false);

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

struct GCS_API PointOnLineConstraint {
    explicit PointOnLineConstraint();

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

struct GCS_API VirtualConstraint {
    explicit VirtualConstraint();

    std::string getTypeName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

template <ConstraintType... Types>
using ConstraintInterface = std::variant<Types...>;

using ConstraintVariant
    = ConstraintInterface<DistanceConstraint, TangencyConstraint,
        AngleConstraint, PointOnLineConstraint, VirtualConstraint>;

class GCS_API Constraint final {
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

    /// @brief Access the underlying constraint variant as a specific
    ///        type. Returns nullptr if the type does not match.
    template <typename T>
    const T* getConstraintAs() const
    {
        return std::get_if<T>(&m_constraint);
    }

    /// @brief Mutable access to the underlying constraint variant.
    template <typename T>
    T* getConstraintAs()
    {
        return std::get_if<T>(&m_constraint);
    }

    std::string getConstraintName() const;
    std::expected<double, ConstraintError> getConstraintValue() const;
};

} // namespace Gcs

#endif // GCS_MODEL_CONSTRAINTS_HPP
