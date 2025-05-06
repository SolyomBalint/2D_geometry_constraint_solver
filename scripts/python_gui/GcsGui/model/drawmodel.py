import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

import cairo

from ..common import commondatastructs as common
from . import gcs


@dataclass
class CanvasCoord:
    x: float
    y: float


class ShapeManager:
    def __init__(self, gcs):
        self.shape_buffer: list = []
        self.gcs: gcs.GeometricConstraintSystem = gcs


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


class Point(Drawable):
    def __init__(self, x: float, y: float, line_width: float, colour: common.RgbColour, radius: float = 5):
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
        if math.sqrt(math.pow(self.coords.x - point.x, 2) + math.pow(self.coords.y - point.y, 2)) <= self.point_radius:
            return True
        return False


class Line(Drawable):
    def __init__(self, x: Point, y: Point, line_width: float, colour: common.RgbColour):
        super().__init__(line_width, colour)
        self.defining_point_one = x
        self.defining_point_two = y

    def on_draw(self, wid, cr: cairo.Context):
        cr.set_line_width(self.line_width)
        cr.set_source_rgb(self.colour.red, self.colour.green, self.colour.blue)

        cr.save()

        cr.move_to(self.defining_point_one.coords.x, self.defining_point_one.coords.y)
        cr.line_to(self.defining_point_two.coords.x, self.defining_point_two.coords.y)
        cr.stroke()

        cr.restore()

    def is_hit_by_point(self, point: CanvasCoord) -> bool:
        # check if point is between line segment (the line is not infinite)
        if (
            min(self.defining_point_one.coords.x, self.defining_point_two.coords.x)
            <= point.x
            <= max(self.defining_point_one.coords.x, self.defining_point_two.coords.x)
        ) and (
            min(self.defining_point_one.coords.y, self.defining_point_two.coords.y)
            <= point.y
            <= max(self.defining_point_one.coords.y, self.defining_point_two.coords.y)
        ):

            # Calculate the cross product: (px - x₁)(y₂ - y₁) - (py - y₁)(x₂ - x₁)
            cross_product: float = (point.x - self.defining_point_one.coords.x) * (
                self.defining_point_two.coords.y - self.defining_point_one.coords.y
            ) - (point.y - self.defining_point_one.coords.y) * (
                self.defining_point_two.coords.x - self.defining_point_one.coords.x
            )

            # We use the line width as the possible epsilon error
            if cross_product < self.line_width:
                return True

        return False


class Circle(Drawable):
    def __init__(self, center: CanvasCoord, radius: float, line_width: float, colour: common.RgbColour):
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
        self.radius = math.sqrt(math.pow(self.center.x - point.x, 2) + math.pow(self.center.y - point.y, 2))

    def is_hit_by_point(self, point: CanvasCoord) -> bool:
        # Checking if distance of point from center is smaller than radius
        if math.sqrt(math.pow(self.center.x - point.x, 2) + math.pow(self.center.y - point.y, 2)) <= self.radius:
            return True
        return False
