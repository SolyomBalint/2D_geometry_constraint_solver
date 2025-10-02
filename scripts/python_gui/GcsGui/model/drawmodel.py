import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

import cairo

from ..common import commondatastructs as common


@dataclass
class CanvasCoord:
    x: float
    y: float


class ShapeManager:
    def __init__(self):
        self.shape_buffer: list = []


class Drawable(ABC):
    def __init__(self, line_width: float, colour: common.RgbColour):
        self.colour: common.RgbColour = colour
        self.line_width = line_width

    @abstractmethod
    def on_draw(self, wid, cr: cairo.Context):
        pass

    @abstractmethod
    def is_hit_by_point(self, point: CanvasCoord) -> bool:
        pass

    @abstractmethod
    def to_dict(self) -> dict:
        """Serialize the drawable to a dictionary"""
        pass

    @staticmethod
    @abstractmethod
    def from_dict(data: dict) -> 'Drawable':
        """Deserialize a drawable from a dictionary"""
        pass


class Point(Drawable):
    def __init__(
        self,
        x: float,
        y: float,
        line_width: float,
        colour: common.RgbColour,
        radius: float = 5,
    ):
        super().__init__(line_width, colour)
        self.coords: CanvasCoord = CanvasCoord(x, y)
        self.point_radius = radius

    def on_draw(self, wid, cr: cairo.Context):
        cr.set_line_width(self.line_width)
        cr.set_source_rgb(self.colour.red, self.colour.green, self.colour.blue)

        # Save the current state of the context
        cr.save()

        # Draw circle(point) at the correct position without translation
        cr.arc(self.coords.x, self.coords.y, self.point_radius, 0, 2 * math.pi)
        cr.stroke_preserve()
        cr.fill()

        # Restore the context to its original state
        cr.restore()

    def is_hit_by_point(self, point: CanvasCoord) -> bool:
        # Checking if distance of point from center is smaller than radius
        if (
            math.sqrt(
                math.pow(self.coords.x - point.x, 2)
                + math.pow(self.coords.y - point.y, 2)
            )
            <= self.point_radius
        ):
            return True
        return False

    def to_dict(self) -> dict:
        """Serialize Point to dictionary"""
        return {
            "type": "Point",
            "x": self.coords.x,
            "y": self.coords.y,
            "line_width": self.line_width,
            "colour": {"r": self.colour.red, "g": self.colour.green, "b": self.colour.blue},
            "radius": self.point_radius
        }

    @staticmethod
    def from_dict(data: dict) -> 'Point':
        """Deserialize Point from dictionary"""
        colour = common.RgbColour(
            data["colour"]["r"],
            data["colour"]["g"],
            data["colour"]["b"]
        )
        return Point(
            data["x"],
            data["y"],
            data["line_width"],
            colour,
            data.get("radius", 5)
        )


class Line(Drawable):
    def __init__(
        self, x: Point, y: Point, line_width: float, colour: common.RgbColour
    ):
        super().__init__(line_width, colour)
        self.defining_point_one = x
        self.defining_point_two = y

    def on_draw(self, wid, cr: cairo.Context):
        cr.set_line_width(self.line_width)
        cr.set_source_rgb(self.colour.red, self.colour.green, self.colour.blue)

        cr.save()

        cr.move_to(
            self.defining_point_one.coords.x, self.defining_point_one.coords.y
        )
        cr.line_to(
            self.defining_point_two.coords.x, self.defining_point_two.coords.y
        )
        cr.stroke()

        cr.restore()

    def is_hit_by_point(self, point: CanvasCoord) -> bool:
        # check if point is between line segment (the line is not infinite)
        if (
            min(
                self.defining_point_one.coords.x,
                self.defining_point_two.coords.x,
            )
            <= point.x
            <= max(
                self.defining_point_one.coords.x,
                self.defining_point_two.coords.x,
            )
        ) and (
            min(
                self.defining_point_one.coords.y,
                self.defining_point_two.coords.y,
            )
            <= point.y
            <= max(
                self.defining_point_one.coords.y,
                self.defining_point_two.coords.y,
            )
        ):

            # Calculate the cross product: (px - x₁)(y₂ - y₁) - (py - y₁)(x₂ - x₁)
            cross_product: float = (
                point.x - self.defining_point_one.coords.x
            ) * (
                self.defining_point_two.coords.y
                - self.defining_point_one.coords.y
            ) - (
                point.y - self.defining_point_one.coords.y
            ) * (
                self.defining_point_two.coords.x
                - self.defining_point_one.coords.x
            )

            # We use the line width as the possible epsilon error
            if cross_product < self.line_width:
                return True

        return False

    def to_dict(self) -> dict:
        """Serialize Line to dictionary"""
        return {
            "type": "Line",
            "point_one": self.defining_point_one.to_dict(),
            "point_two": self.defining_point_two.to_dict(),
            "line_width": self.line_width,
            "colour": {"r": self.colour.red, "g": self.colour.green, "b": self.colour.blue}
        }

    @staticmethod
    def from_dict(data: dict) -> 'Line':
        """Deserialize Line from dictionary"""
        colour = common.RgbColour(
            data["colour"]["r"],
            data["colour"]["g"],
            data["colour"]["b"]
        )
        point_one = Point.from_dict(data["point_one"])
        point_two = Point.from_dict(data["point_two"])
        return Line(point_one, point_two, data["line_width"], colour)


class Circle(Drawable):
    def __init__(
        self,
        center: CanvasCoord,
        radius: float,
        line_width: float,
        colour: common.RgbColour,
    ):
        super().__init__(line_width, colour)
        self.center: CanvasCoord = center
        self.radius: float = radius

    def on_draw(self, wid, cr: cairo.Context):
        cr.set_line_width(self.line_width)
        cr.set_source_rgb(self.colour.red, self.colour.green, self.colour.blue)

        # Save the current state of the context
        cr.save()

        # Draw circle(point) at the correct position without translation
        cr.arc(self.center.x, self.center.y, self.radius, 0, 2 * math.pi)
        cr.stroke()

        # Restore the context to its original state
        cr.restore()

    def calculate_and_set_radius(self, point: CanvasCoord):
        self.radius = math.sqrt(
            math.pow(self.center.x - point.x, 2)
            + math.pow(self.center.y - point.y, 2)
        )

    def is_hit_by_point(self, point: CanvasCoord) -> bool:
        # Checking if distance of point from center is smaller than radius
        if (
            math.sqrt(
                math.pow(self.center.x - point.x, 2)
                + math.pow(self.center.y - point.y, 2)
            )
            <= self.radius
        ):
            return True
        return False

    def to_dict(self) -> dict:
        """Serialize Circle to dictionary"""
        return {
            "type": "Circle",
            "center_x": self.center.x,
            "center_y": self.center.y,
            "radius": self.radius,
            "line_width": self.line_width,
            "colour": {"r": self.colour.red, "g": self.colour.green, "b": self.colour.blue}
        }

    @staticmethod
    def from_dict(data: dict) -> 'Circle':
        """Deserialize Circle from dictionary"""
        colour = common.RgbColour(
            data["colour"]["r"],
            data["colour"]["g"],
            data["colour"]["b"]
        )
        center = CanvasCoord(data["center_x"], data["center_y"])
        return Circle(center, data["radius"], data["line_width"], colour)
