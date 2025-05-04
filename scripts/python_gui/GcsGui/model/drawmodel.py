import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

import cairo

from ..common import commondatastructs as common


@dataclass
class CanvasCoord:
    x: float
    y: float


class Drawable(ABC):
    def __init__(self, line_width: float, colour: common.RgbColour):
        self.colour: common.RgbColour = colour
        self.line_width = line_width

    @abstractmethod
    def on_draw(self, wid, cr: cairo.Context):
        pass

    def is_hit_by_point(self, x: float, y: float):
        raise NotImplementedError(f"The point collosion function is not implemented for class: {type(self)}")


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
        # cr.fill()

        # Restore the context to its original state
        cr.restore()

    def calculate_and_set_radius(self, p: CanvasCoord):
        self.radius = math.sqrt(math.pow(self.center.x - p.x, 2) + math.pow(self.center.y - p.y, 2))
