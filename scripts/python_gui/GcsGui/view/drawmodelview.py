import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
import math
from enum import IntEnum

import cairo
from gi.repository import Gdk, Gio, Gtk

from ..common import commondatastructs
from ..model import drawmodel


class DrawingCanvasWidget(Gtk.DrawingArea):

    def __init__(self):
        super(DrawingCanvasWidget, self).__init__()

        self.point_buffer: list[drawmodel.CanvasCoord] = []
        self.connect("draw", self.on_draw)

        self.set_hexpand(True)
        self.set_vexpand(True)
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

        if e.type == Gdk.EventType.BUTTON_PRESS and e.button == commondatastructs.MouseButtonGtkId.LEFT_MOUSE_BUTTON:

            self.point_buffer.append(drawmodel.CanvasCoord(e.x, e.y))

            self.queue_draw()

    def clear_canvas(self):
        self.point_buffer.clear()
        self.queue_draw()


class DrawingLayout(Gtk.Grid):
    def __init__(self):
        super().__init__()
        self.set_column_homogeneous(True)
        self.set_row_homogeneous(True)

        list_box = Gtk.ListBox()
        list_box.set_selection_mode(Gtk.SelectionMode.NONE)

        draw_methods = [
            "Point",
            "Line",
            "Circle",
        ]
        drawing_combo = Gtk.ComboBoxText()
        drawing_combo.set_entry_text_column(0)
        drawing_combo.connect("changed", self.on_drawing_method_changed)
        for method in draw_methods:
            drawing_combo.append_text(method)

        drawing_combo.set_active(0)

        draw_methods_row = Gtk.ListBoxRow()
        draw_methods_row.add(drawing_combo)

        list_box.add(draw_methods_row)

        # Add rgb setter
        self.createRgbSpinButton(self.on_red_colour_change, list_box, "Set Red Color")
        self.createRgbSpinButton(self.on_green_colour_change, list_box, "Set Green Color")
        self.createRgbSpinButton(self.on_blue_colour_change, list_box, "Set Blue Color")

        # Add a clear button to the header
        clear_button = Gtk.Button.new_with_label("Clear Canvas")
        clear_button.connect("clicked", self.on_click_clear_draw)
        clear_row = Gtk.ListBoxRow()
        clear_row.add(clear_button)

        list_box.add(clear_row)

        self.drawing_area = DrawingCanvasWidget()
        self.add(list_box)
        self.attach(self.drawing_area, 1, 0, 5, 1)

    def createRgbSpinButton(self, value_changed_function, list_box, label):

        box = Gtk.Box(spacing=5)

        adjustment = Gtk.Adjustment(value=0, lower=0, upper=255, step_increment=1, page_increment=10, page_size=0)
        spinbutton_color = Gtk.SpinButton()
        spinbutton_color.set_adjustment(adjustment)
        spinbutton_color.set_numeric(True)
        spinbutton_color.connect("value-changed", value_changed_function)

        box.add(spinbutton_color)

        label = Gtk.Label(label=label)
        label.set_justify(Gtk.Justification.RIGHT)

        box.add(label)

        row = Gtk.ListBoxRow()
        row.add(box)
        list_box.add(row)

    # TODO
    def on_drawing_method_changed(self, combo):
        return

    # TODO
    def on_red_colour_change(self, scroll):
        print(scroll.get_value())
        return

    # TODO
    def on_green_colour_change(self, scroll):
        return

    # TODO
    def on_blue_colour_change(self, scroll):
        return

    def on_click_clear_draw(self, button):
        self.drawing_area.clear_canvas()
