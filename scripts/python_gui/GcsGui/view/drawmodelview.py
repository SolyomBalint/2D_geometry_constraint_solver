import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
import copy
from enum import StrEnum

import cairo
from gi.repository import Gdk, Gio, Gtk

from ..common import commondatastructs as common
from ..model import drawmodel


class DrawingMethod(StrEnum):
    POINT = "Point"
    LINE = "Line"
    CIRCLE = "Circle"

    @classmethod
    def get_enum_based_on_str(cls, inputMethod: str):
        for method in DrawingMethod:
            if method == inputMethod:
                return method
        raise ValueError("Non existent StrEnum input")


class DrawingCanvasWidget(Gtk.DrawingArea):

    def __init__(self):
        super(DrawingCanvasWidget, self).__init__()

        self.shape_buffer = []  # This will contain drawmodel.Drawable instances
        self.current_colour: common.RgbColour = common.RgbColour(0, 0, 0)
        self.current_width: float = 5
        self.drawing_method: DrawingMethod = DrawingMethod.POINT  # Based on DrawingLayout combobox initialization
        self.temp_start_point: bool = False
        self.connect("draw", self.on_draw)

        self.set_hexpand(True)
        self.set_vexpand(True)
        self.set_events(
            Gdk.EventMask.BUTTON_PRESS_MASK
            | Gdk.EventMask.BUTTON_RELEASE_MASK
            | Gdk.EventMask.BUTTON1_MOTION_MASK
            # | Gdk.EventMask.POINTER_MOTION_MASK  # Receives all mouse motion regardless of event
        )  # Mouse events have to be enabled manually
        self.connect("button-press-event", self.on_button_press)
        self.connect("button-release-event", self.on_button_release)
        self.connect("motion-notify-event", self.on_motion_notify)

    def on_draw(self, wid, cr: cairo.Context):

        for shape in self.shape_buffer:
            shape.on_draw(wid, cr)

    def on_button_press(self, widget, event):

        if event.type == Gdk.EventType.BUTTON_PRESS and event.button == common.MouseButtonGtkId.LEFT_MOUSE_BUTTON:

            match self.drawing_method:
                case DrawingMethod.POINT:
                    self.shape_buffer.append(
                        drawmodel.Point(event.x, event.y, self.current_width, copy.deepcopy(self.current_colour))
                    )

                    self.queue_draw()

                case DrawingMethod.LINE:
                    if not self.temp_start_point:
                        self.temp_start_point = True
                        self.shape_buffer.append(
                            drawmodel.Point(event.x, event.y, self.current_width, copy.deepcopy(self.current_colour))
                        )
                    else:
                        self.shape_buffer.append(
                            drawmodel.Line(
                                self.shape_buffer[-1],
                                drawmodel.Point(
                                    event.x,
                                    event.y,
                                    copy.deepcopy(self.current_width),
                                    copy.deepcopy(self.current_colour),
                                ),
                                copy.deepcopy(self.current_width),
                                copy.deepcopy(self.current_colour),
                            )
                        )
                        self.shape_buffer.append(self.shape_buffer[-1].defining_point_two)

                        self.temp_start_point = False

                    self.queue_draw()
                case DrawingMethod.CIRCLE:
                    self.shape_buffer.append(
                        drawmodel.Circle(
                            drawmodel.CanvasCoord(event.x, event.y),
                            0.0,
                            self.current_width,
                            copy.deepcopy(self.current_colour),
                        )
                    )

    def on_motion_notify(self, widget, event):
        match self.drawing_method:
            case DrawingMethod.CIRCLE:
                self.set_circle_radius_and_draw(drawmodel.CanvasCoord(event.x, event.y))

    def on_button_release(self, widget, event):
        if event.type == Gdk.EventType.BUTTON_RELEASE and event.button == common.MouseButtonGtkId.LEFT_MOUSE_BUTTON:
            match self.drawing_method:
                case DrawingMethod.CIRCLE:
                    self.set_circle_radius_and_draw(drawmodel.CanvasCoord(event.x, event.y))

    def set_circle_radius_and_draw(self, p: drawmodel.CanvasCoord):
        self.shape_buffer[-1].calculate_and_set_radius(drawmodel.CanvasCoord(p.x, p.y))
        self.queue_draw()

    def clear_canvas(self):
        self.shape_buffer.clear()
        self.queue_draw()

    def setDrawingMethod(self, stringRep: str):
        self.drawing_method = DrawingMethod.get_enum_based_on_str(stringRep)


# TODO circle, collosion detection
class DrawingLayout(Gtk.Grid):
    def __init__(self):
        super().__init__()
        self.set_column_homogeneous(True)
        self.set_row_homogeneous(True)

        self.drawing_area = DrawingCanvasWidget()

        list_box = Gtk.ListBox()
        list_box.set_selection_mode(Gtk.SelectionMode.NONE)

        draw_methods = [
            DrawingMethod.POINT,
            DrawingMethod.LINE,
            DrawingMethod.CIRCLE,
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
        self.createSpinButton(
            self.on_red_colour_change,
            list_box,
            "Set Red Color",
            Gtk.Adjustment(value=0, lower=0, upper=1, step_increment=0.01, page_increment=0.1, page_size=0),
        )
        self.createSpinButton(
            self.on_green_colour_change,
            list_box,
            "Set Green Color",
            Gtk.Adjustment(value=0, lower=0, upper=1, step_increment=0.01, page_increment=0.1, page_size=0),
        )
        self.createSpinButton(
            self.on_blue_colour_change,
            list_box,
            "Set Blue Color",
            Gtk.Adjustment(value=0, lower=0, upper=1, step_increment=0.01, page_increment=0.1, page_size=0),
        )

        # Line width setter
        self.createSpinButton(
            self.set_line_width,
            list_box,
            "Set line width",
            Gtk.Adjustment(value=1, lower=1, upper=30, step_increment=1, page_increment=10, page_size=0),
            0,
        )

        # Add a clear button to the header
        clear_button = Gtk.Button.new_with_label("Clear Canvas")
        clear_button.connect("clicked", self.on_click_clear_draw)
        clear_row = Gtk.ListBoxRow()
        clear_row.add(clear_button)

        list_box.add(clear_row)

        self.add(list_box)
        self.attach(self.drawing_area, 1, 0, 5, 1)

    def createSpinButton(
        self,
        value_changed_function,
        list_box,
        label,
        adjustment,
        decimal_points=2,
    ):

        box = Gtk.Box(spacing=5)

        spinbutton_color = Gtk.SpinButton()
        spinbutton_color.set_adjustment(adjustment)
        spinbutton_color.set_numeric(True)
        spinbutton_color.set_digits(decimal_points)
        spinbutton_color.connect("value-changed", value_changed_function)

        box.add(spinbutton_color)

        label = Gtk.Label(label=label)
        label.set_justify(Gtk.Justification.RIGHT)

        box.add(label)

        row = Gtk.ListBoxRow()
        row.add(box)
        list_box.add(row)

    def on_drawing_method_changed(self, combo):
        self.drawing_area.setDrawingMethod(combo.get_active_text())

    def on_red_colour_change(self, scroll):
        self.drawing_area.current_colour.red = scroll.get_value()

    def on_green_colour_change(self, scroll):
        self.drawing_area.current_colour.green = scroll.get_value()

    def on_blue_colour_change(self, scroll):
        self.drawing_area.current_colour.blue = scroll.get_value()

    def on_click_clear_draw(self, button):
        self.drawing_area.clear_canvas()

    def set_line_width(self, scroll):
        self.drawing_area.current_width = scroll.get_value()
