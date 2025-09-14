#ifndef ELEMENTS_HPP
#define ELEMENTS_HPP

#include <string>
#include <variant>

namespace Gcs {

template <typename T>
concept ElementType = requires(T element) {
    { element.getTypeName() } -> std::convertible_to<std::string>;
    // TODO function concept for updateElementPosition
};

struct Point {
    float x, y;
    explicit Point(float x_coord, float y_coord)
        : x { x_coord }
        , y { y_coord }
    {
    }
    std::string getTypeName() const { return "Point"; }
    void updateElementPosition(float new_x, float new_y)
    {
        x = new_x;
        y = new_y;
    }
};

struct FixedRadiusCircle {
    float x, y;
    float fixed_radius;
    explicit FixedRadiusCircle(float x_coord, float y_coord, float r)
        : x { x_coord }
        , y { y_coord }
        , fixed_radius { r }
    {
    }
    std::string getTypeName() const { return "FixedRadiusCircle"; }
    void updateElementPosition(float new_x, float new_y)
    {
        x = new_x;
        y = new_y;
    }
};

struct Line {
    float r0_x, r0_y;
    float v_x, v_y;
    explicit Line(
        float r0_x_coord, float r0_y_coord, float v_x_dir, float v_y_dir)
        : r0_x { r0_x_coord }
        , r0_y { r0_y_coord }
        , v_x { v_x_dir }
        , v_y { v_y_dir }
    {
    }
    std::string getTypeName() const { return "Line"; }
    void updateElementPosition(
        float new_r0_x, float new_r0_y, float new_v_x, float new_v_y)
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

    template <typename... Parameters>
    void updateElementPosition(Parameters&&... params)
    {
        std::visit(
            [&params...](auto&& element) {
                element.updateElementPosition(
                    std::forward<Parameters>(params)...);
            },
            element);
    }
};
} // namespace Gcs

#endif // ELEMENTS_HPP
