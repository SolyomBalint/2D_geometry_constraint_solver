#!/usr/bin/env python3

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
import math
from dataclasses import dataclass
from enum import IntEnum

import cairo
from gi.repository import Gdk, Gtk
from graph_tool.all import *


def own_small_example():
    indirected_graph = Graph(directed=False)
    vertice1 = indirected_graph.add_vertex()
    vertice2 = indirected_graph.add_vertex()
    vertice3 = indirected_graph.add_vertex()

    edge1 = indirected_graph.add_edge(vertice1, vertice2)
    edge2 = indirected_graph.add_edge(vertice1, vertice3)
    edge3 = indirected_graph.add_edge(vertice3, vertice2)

    K = 0.5
    pos = sfdp_layout(indirected_graph, K=K)
    return GraphWidget(indirected_graph, pos)


class MouseButtonGtkId(IntEnum):
    LEFT_MOUSE_BUTTON = 1  # These numbers are assigned in GTK to the mouse buttons
    RIGHT_MOUSE_BUTTON = 3


@dataclass
class CanvasCoord:
    x: float
    y: float


class DrawingCanvasWidget(Gtk.DrawingArea):

    def __init__(self):
        super(DrawingCanvasWidget, self).__init__()

        self.point_buffer: list[CanvasCoord] = []
        self.connect("draw", self.on_draw)

        self.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)  # Mouse events have to be enabled manually
        self.connect("button-press-event", self.on_button_press)

    def on_draw(self, wid, cr):
        cr.set_line_width(5)
        cr.set_source_rgb(0, 0, 0)

        for circle in self.point_buffer:
            # Save the current state of the context
            cr.save()

            # Draw circle at the correct position without translation
            cr.arc(circle.x, circle.y, 5, 0, 2 * math.pi)
            cr.stroke_preserve()
            cr.fill()

            # Restore the context to its original state
            cr.restore()

    def on_button_press(self, w, e):

        if e.type == Gdk.EventType.BUTTON_PRESS and e.button == MouseButtonGtkId.LEFT_MOUSE_BUTTON:

            self.point_buffer.append(CanvasCoord(e.x, e.y))

            self.queue_draw()

    def clear_canvas(self):
        self.point_buffer.clear()
        self.queue_draw()


class MainGtkWindow(Gtk.Window):

    def __init__(self):

        super().__init__(title="Geometric Constraint Solver")

        # Set up the main window
        self.set_default_size(1920, 1200)
        self.connect("delete-event", Gtk.main_quit)

        # Create grid layout
        self.grid = Gtk.Grid()
        self.grid.set_column_homogeneous(True)
        self.grid.set_row_homogeneous(True)
        self.add(self.grid)

        # Create drawing area
        self.drawing_area = DrawingCanvasWidget()

        # Make the drawing area expand to fill available space
        self.drawing_area.set_hexpand(True)
        self.drawing_area.set_vexpand(True)

        # Add drawing area to grid (spanning 2 columns and rows for example)
        self.grid.attach(self.drawing_area, 0, 0, 2, 2)
        self.grid.attach_next_to(own_small_example(), self.drawing_area, Gtk.PositionType.RIGHT, 2, 2)

        # # Add some example controls to demonstrate grid layout
        # clear_button = Gtk.Button(label="Clear Drawing")
        # clear_button.connect("clicked", self.on_clear_clicked)
        # self.grid.attach(clear_button, 0, 2, 1, 1)
        #
        # info_label = Gtk.Label(label="Left-click to place circles, Right-click to select")
        # self.grid.attach(info_label, 1, 2, 1, 1)

        # Show all widgets
        self.show_all()

    def on_clear_clicked(self, button):
        self.drawing_area.clear_canvas()


def main():

    app = MainGtkWindow()
    Gtk.main()


if __name__ == "__main__":
    main()
