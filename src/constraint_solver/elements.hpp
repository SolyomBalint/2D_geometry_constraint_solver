#ifndef ELEMENTS_HPP
#define ELEMENTS_HPP

#include <string>
#include <variant>

namespace Gcs {

template <typename T>
concept ElementType = requires(T element) {
    { element.getTypeName() } -> std::convertible_to<std::string>;
    { element.toString() } -> std::convertible_to<std::string>;
};

struct Point {
    double OiriginalXOnCanvas;
    double OriginalYOnCanvas;
    double x, y;

    Point();
    explicit Point(double xCoordCanvas, double yCoordCanvas);

    std::string getTypeName() const;
    std::string toString() const;
    void updateElementPosition(double new_x, double new_y);
};

struct FixedRadiusCircle {
    double x, y;
    double fixed_radius;

    FixedRadiusCircle();
    explicit FixedRadiusCircle(double x_coord, double y_coord, double r);

    std::string getTypeName() const;
    std::string toString() const;
    void updateElementPosition(double new_x, double new_y);
};

struct Line {
    double originalX1OnCanvas;
    double originalY1OnCanvas;
    double originalX2OnCanvas;
    double originalY2OnCanvas;
    double x1, y1;
    double x2, y2;

    Line();
    explicit Line(double, double, double, double);

    std::string getTypeName() const;
    std::string toString() const;
    void updateElementPosition(
        double new_x1, double new_y1, double new_x2, double new_y2);
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

    std::string getElementName() const;
    std::string toString() const;

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
