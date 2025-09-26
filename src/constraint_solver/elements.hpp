#ifndef ELEMENTS_HPP
#define ELEMENTS_HPP

#include <string>
#include <variant>
#include <format>

namespace Gcs {

template <typename T>
concept ElementType = requires(T element) {
    { element.getTypeName() } -> std::convertible_to<std::string>;
    { element.toString() } -> std::convertible_to<std::string>;
};

struct Point {
    double x, y;
    Point() { };
    explicit Point(double x_coord, double y_coord)
        : x { x_coord }
        , y { y_coord }
    {
    }
    std::string getTypeName() const { return "Point"; }
    std::string toString() const
    {
        return std::format("Point(x: {}, y: {})", x, y);
    }
    void updateElementPosition(double new_x, double new_y)
    {
        x = new_x;
        y = new_y;
    }
};

struct FixedRadiusCircle {
    double x, y;
    double fixed_radius;
    FixedRadiusCircle() { };
    explicit FixedRadiusCircle(double x_coord, double y_coord, double r)
        : x { x_coord }
        , y { y_coord }
        , fixed_radius { r }
    {
    }
    std::string getTypeName() const { return "FixedRadiusCircle"; }
    std::string toString() const
    {
        return std::format(
            "FixedRadiusCircle(x: {}, y: {}, radius: {})", x, y, fixed_radius);
    }
    void updateElementPosition(double new_x, double new_y)
    {
        x = new_x;
        y = new_y;
    }
};

struct Line {
    double r0_x, r0_y;
    double v_x, v_y;
    Line() { };
    explicit Line(
        double r0_x_coord, double r0_y_coord, double v_x_dir, double v_y_dir)
        : r0_x { r0_x_coord }
        , r0_y { r0_y_coord }
        , v_x { v_x_dir }
        , v_y { v_y_dir }
    {
    }
    std::string getTypeName() const { return "Line"; }
    std::string toString() const
    {
        return std::format(
            "Line(r0: ({}, {}), v: ({}, {}))", r0_x, r0_y, v_x, v_y);
    }
    void updateElementPosition(
        double new_r0_x, double new_r0_y, double new_v_x, double new_v_y)
    {
        r0_x = new_r0_x;
        r0_y = new_r0_y;
        v_x = new_v_x;
        v_y = new_v_y;
    }
};

template <ElementType... Types> using ElementInterface = std::variant<Types...>;

class Element final {
private:
    ElementInterface<Point, FixedRadiusCircle, Line> element;
    bool isSet = false;

public:
    template <typename T>
    explicit Element()
        : element { T {} }
    {
    }

    template <typename T>
    explicit Element(const T& e)
        : element { e }
    {
    }
    template <typename T> bool isElementType() const
    {
        return std::holds_alternative<T>(element);
    }

    template <typename Visitor> auto visitElement(Visitor&& visitor)
    {
        return std::visit(visitor, element);
    }

    std::string getElementName() const
    {
        return std::visit(
            [](const auto& element) { return element.getTypeName(); }, element);
    }

    std::string toString() const
    {
        return std::visit(
            [](const auto& element) { return element.toString(); }, element);
    }

    template <typename T> T& getElement() { return std::get<T>(element); }

    template <typename T> const T& getElement() const
    {
        return std::get<T>(element);
    }

    bool isElementSet() const { return isSet; }

    template <typename... Parameters>
    void updateElementPosition(Parameters&&... params)
    {
        std::visit(
            [&](auto&& element) -> void {
                if constexpr (requires {
                                  element.updateElementPosition(
                                      std::forward<Parameters>(params)...);
                              }) {
                    element.updateElementPosition(
                        std::forward<Parameters>(params)...);
                }
            },
            element);
        isSet = true;
    }
};
} // namespace Gcs

#endif // ELEMENTS_HPP
