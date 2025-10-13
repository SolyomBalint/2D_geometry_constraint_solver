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
    def __init__(
        self, line_width: float, colour: common.RgbColour, visible: bool = True
    ):
        self.colour: common.RgbColour = colour
        self.line_width = line_width
        self.visible = visible

    @abstractmethod
    def on_draw(self, wid, cr: cairo.Context):
        pass

    @abstractmethod
    def is_hit_by_point(self, point: CanvasCoord) -> bool:
        pass

    @abstractmethod
    def to_dict(self, **kwargs) -> dict:
        """Serialize the drawable to a dictionary"""
        pass

    @staticmethod
    @abstractmethod
    def from_dict(data: dict) -> "Drawable":
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
        visible: bool = True,
    ):
        super().__init__(line_width, colour, visible)
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

    def to_dict(self, **kwargs) -> dict:
        """Serialize Point to dictionary"""
        return {
            "type": "Point",
            "x": self.coords.x,
            "y": self.coords.y,
            "line_width": self.line_width,
            "colour": {
                "r": self.colour.red,
                "g": self.colour.green,
                "b": self.colour.blue,
            },
            "radius": self.point_radius,
            "visible": self.visible,
        }

    @staticmethod
    def from_dict(data: dict) -> "Point":
        """Deserialize Point from dictionary"""
        colour = common.RgbColour(
            data["colour"]["r"], data["colour"]["g"], data["colour"]["b"]
        )
        return Point(
            data["x"],
            data["y"],
            data["line_width"],
            colour,
            data.get("radius", 5),
            data.get("visible", True),
        )


class Line(Drawable):
    def __init__(
        self,
        x: Point,
        y: Point,
        line_width: float,
        colour: common.RgbColour,
        visible: bool = True,
        add_to_constraint_graph: bool = True,
    ):
        super().__init__(line_width, colour, visible)
        self.defining_point_one = x
        self.defining_point_two = y
        self.add_to_constraint_graph = (
            add_to_constraint_graph  # Flag for visual-only lines
        )

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
        """
        Check if a given point hits the line segment, considering line width.
        """

        x1, y1 = (
            self.defining_point_one.coords.x,
            self.defining_point_one.coords.y,
        )
        x2, y2 = (
            self.defining_point_two.coords.x,
            self.defining_point_two.coords.y,
        )
        px, py = point.x, point.y

        # Vector from point 1 → point 2
        dx, dy = x2 - x1, y2 - y1
        line_len_sq = dx * dx + dy * dy
        if line_len_sq == 0:
            # Degenerate case: both endpoints are the same
            dist_sq = (px - x1) ** 2 + (py - y1) ** 2
            return dist_sq <= (self.line_width / 2) ** 2

        # Project point onto the line (clamp t to segment [0,1])
        t = ((px - x1) * dx + (py - y1) * dy) / line_len_sq
        t = max(0.0, min(1.0, t))

        # Closest point on the segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        # Distance from the point to the line
        dist_sq = (px - closest_x) ** 2 + (py - closest_y) ** 2

        # Compare to half of the line width (for hit tolerance)
        return dist_sq <= (self.line_width / 2) ** 2

    def to_dict(self, **kwargs) -> dict:
        """Serialize Line to dictionary"""
        shape_to_index = kwargs.get('shape_to_index')

        # If shape_to_index is provided, use indices; otherwise use nested objects (backward compatibility)
        if shape_to_index is not None:
            # Store indices to the defining points
            point_one_index = shape_to_index.get(self.defining_point_one, -1)
            point_two_index = shape_to_index.get(self.defining_point_two, -1)

            return {
                "type": "Line",
                "point_one_index": point_one_index,
                "point_two_index": point_two_index,
                "line_width": self.line_width,
                "colour": {
                    "r": self.colour.red,
                    "g": self.colour.green,
                    "b": self.colour.blue,
                },
                "visible": self.visible,
                "add_to_constraint_graph": self.add_to_constraint_graph,
            }
        else:
            # Fallback to nested objects for backward compatibility
            return {
                "type": "Line",
                "point_one": self.defining_point_one.to_dict(),
                "point_two": self.defining_point_two.to_dict(),
                "line_width": self.line_width,
                "colour": {
                    "r": self.colour.red,
                    "g": self.colour.green,
                    "b": self.colour.blue,
                },
                "visible": self.visible,
                "add_to_constraint_graph": self.add_to_constraint_graph,
            }

    @staticmethod
    def from_dict(data: dict, point_map: dict = None) -> "Line":
        """Deserialize Line from dictionary

        Args:
            data: Dictionary containing line data
            point_map: Optional mapping from shape index to Point objects
        """
        colour = common.RgbColour(
            data["colour"]["r"], data["colour"]["g"], data["colour"]["b"]
        )

        # Check if we have point indices (new format) and a point_map
        if "point_one_index" in data and "point_two_index" in data and point_map is not None:
            # Use indices to reference actual Point objects from point_map
            point_one_index = data["point_one_index"]
            point_two_index = data["point_two_index"]

            point_one = point_map.get(point_one_index)
            point_two = point_map.get(point_two_index)

            if point_one is None or point_two is None:
                raise ValueError(f"Invalid point indices: {point_one_index}, {point_two_index}")
        else:
            # Fallback to nested objects (old format or no point_map provided)
            point_one = Point.from_dict(data["point_one"])
            point_two = Point.from_dict(data["point_two"])

        return Line(
            point_one,
            point_two,
            data["line_width"],
            colour,
            data.get("visible", True),
            data.get(
                "add_to_constraint_graph", True
            ),  # Default True for backward compatibility
        )


class Circle(Drawable):
    def __init__(
        self,
        center: CanvasCoord,
        radius: float,
        line_width: float,
        colour: common.RgbColour,
        visible: bool = True,
    ):
        super().__init__(line_width, colour, visible)
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

    def to_dict(self, **kwargs) -> dict:
        """Serialize Circle to dictionary"""
        return {
            "type": "Circle",
            "center_x": self.center.x,
            "center_y": self.center.y,
            "radius": self.radius,
            "line_width": self.line_width,
            "colour": {
                "r": self.colour.red,
                "g": self.colour.green,
                "b": self.colour.blue,
            },
            "visible": self.visible,
        }

    @staticmethod
    def from_dict(data: dict) -> "Circle":
        """Deserialize Circle from dictionary"""
        colour = common.RgbColour(
            data["colour"]["r"], data["colour"]["g"], data["colour"]["b"]
        )
        center = CanvasCoord(data["center_x"], data["center_y"])
        return Circle(
            center,
            data["radius"],
            data["line_width"],
            colour,
            data.get("visible", True),
        )
