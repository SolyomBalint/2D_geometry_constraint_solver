#ifndef ELEMENTS_HPP
#define ELEMENTS_HPP

// General STD/STL headers
#include <cassert>
#include <concepts>
#include <string>
#include <variant>

// Thirdparty headers
#include <Eigen/Core>

namespace Gcs {

template <typename T>
concept ElementType = requires(T element) {
    { element.getTypeName() } -> std::convertible_to<std::string>;
    { element.toString() } -> std::convertible_to<std::string>;
};

struct Point {
    Eigen::Vector2d canvasPosition; ///< Original position on canvas.
    Eigen::Vector2d position; ///< Solver-calculated position.

    Point();
    explicit Point(const Eigen::Vector2d& canvasPos);

    std::string getTypeName() const;
    std::string toString() const;
    void updateElementPosition(const Eigen::Vector2d& newPosition);
};

struct FixedRadiusCircle {
    Eigen::Vector2d position; ///< Center position.
    double fixedRadius;

    FixedRadiusCircle();
    explicit FixedRadiusCircle(const Eigen::Vector2d& centerPos, double r);

    std::string getTypeName() const;
    std::string toString() const;
    void updateElementPosition(const Eigen::Vector2d& newPosition);
};

struct Line {
    Eigen::Vector2d canvasP1; ///< Original canvas endpoint 1.
    Eigen::Vector2d canvasP2; ///< Original canvas endpoint 2.
    Eigen::Vector2d p1; ///< Solver-calculated endpoint 1.
    Eigen::Vector2d p2; ///< Solver-calculated endpoint 2.

    Line();
    explicit Line(const Eigen::Vector2d& canvasEndpoint1,
        const Eigen::Vector2d& canvasEndpoint2);

    std::string getTypeName() const;
    std::string toString() const;
    void updateElementPosition(
        const Eigen::Vector2d& newP1, const Eigen::Vector2d& newP2);

    /**
     * @brief Direction vector from p1 to p2.
     * @return Unnormalized direction vector @c p2 - @c p1.
     */
    Eigen::Vector2d direction() const;

    /**
     * @brief Unit direction vector from p1 to p2.
     * @return Normalized direction vector.
     * @pre Line must have non-zero length.
     */
    Eigen::Vector2d unitDirection() const;

    /**
     * @brief Normal vector perpendicular to the line direction.
     * @return A vector perpendicular to @c direction(), rotated 90 degrees
     *         counter-clockwise. Not normalized.
     */
    Eigen::Vector2d normal() const;

    /**
     * @brief Euclidean length of the line segment.
     * @return The distance between @c p1 and @c p2.
     */
    double length() const;

    /**
     * @brief Midpoint of the line segment.
     * @return The point halfway between @c p1 and @c p2.
     */
    Eigen::Vector2d midpoint() const;
};

template <ElementType... Types>
using ElementInterface = std::variant<Types...>;

using ElementVariant = ElementInterface<Point, FixedRadiusCircle, Line>;

class Element final {
private:
    ElementVariant m_element;
    bool m_isSet = false;

public:
    template <typename T>
    explicit Element(const T& e)
        : m_element { e }
    {
    }

    template <typename T>
    bool isElementType() const
    {
        return std::holds_alternative<T>(m_element);
    }

    std::string getElementName() const;
    std::string toString() const;

    template <typename T>
    T& getElement()
    {
        return std::get<T>(m_element);
    }

    template <typename T>
    const T& getElement() const
    {
        return std::get<T>(m_element);
    }

    bool isElementSet() const { return m_isSet; }

    template <typename... Parameters>
    void updateElementPosition(Parameters&&... params)
    {
        std::visit(
            [&](auto&& elem) -> void {
                if constexpr (requires {
                                  elem.updateElementPosition(
                                      std::forward<Parameters>(params)...);
                              }) {
                    elem.updateElementPosition(
                        std::forward<Parameters>(params)...);
                    m_isSet = true;
                } else {
                    // NOLINTNEXTLINE(cert-dcl03-c)
                    assert(false
                        && "updateElementPosition called with arguments "
                           "that do not match the active element type's "
                           "signature");
                }
            },
            m_element);
    }
};
} // namespace Gcs

#endif // ELEMENTS_HPP
