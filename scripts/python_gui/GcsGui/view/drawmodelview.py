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

    class ConstraintDialog(Gtk.Dialog):
        def __init__(
            self, parent, title="Enter a value", message="Enter a float value:"
        ):
            super().__init__(
                title=title,
                transient_for=parent,
                flags=Gtk.DialogFlags.MODAL
                | Gtk.DialogFlags.DESTROY_WITH_PARENT,
            )
            self.add_button(Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL)
            self.add_button(Gtk.STOCK_OK, Gtk.ResponseType.OK)

            # Set default size and border width
            self.set_default_size(300, 100)
            self.set_border_width(10)

            # Create container for content
            content_area = self.get_content_area()

            # Create a vertical box to organize widgets
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            content_area.add(box)

            # Add the label
            label = Gtk.Label(label=message)
            label.set_halign(Gtk.Align.START)
            box.pack_start(label, False, False, 0)

            # Add the float entry
            self.entry = Gtk.Entry()
            self.entry.set_activates_default(True)
            box.pack_start(self.entry, True, True, 0)

            # Set the OK button as default
            self.set_default_response(Gtk.ResponseType.OK)

            # Show all widgets
            self.show_all()

        def get_float(self):
            try:
                return float(self.entry.get_text())
            except ValueError:
                return None

    def __init__(self, shape_manager: drawmodel.ShapeManager, gcs_system=None):
        super(DrawingCanvasWidget, self).__init__()

        self.shape_manager = (
            shape_manager  # This will contain drawmodel.Drawable instances
        )
        self.gcs_system = (
            gcs_system  # Reference to the GeometricConstraintSystem
        )
        self.current_colour: common.RgbColour = common.RgbColour(0, 0, 0)
        self.current_width: float = 5
        self.drawing_method: DrawingMethod = (
            DrawingMethod.POINT
        )  # Based on DrawingLayout combobox initialization
        self.temp_start_point: bool = False
        self.selected_items: list = []
        self.show_constraints: bool = False  # Toggle for constraint rendering
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

    def get_shape_center(self, shape):
        """Get the center point of a shape for constraint rendering"""
        if isinstance(shape, drawmodel.Point):
            return (shape.coords.x, shape.coords.y)
        elif isinstance(shape, drawmodel.Circle):
            return (shape.center.x, shape.center.y)
        elif isinstance(shape, drawmodel.Line):
            # Use midpoint of line
            x = (shape.defining_point_one.coords.x + shape.defining_point_two.coords.x) / 2
            y = (shape.defining_point_one.coords.y + shape.defining_point_two.coords.y) / 2
            return (x, y)
        return None

    def get_shape_type_name(self, shape):
        """Get the type name of a shape for labeling"""
        if isinstance(shape, drawmodel.Point):
            return "Point"
        elif isinstance(shape, drawmodel.Circle):
            return "Circle"
        elif isinstance(shape, drawmodel.Line):
            return "Line"
        return "Unknown"

    def draw_constraints(self, widget, cr: cairo.Context):
        """Draw constraint visualizations (dotted lines with labels)"""
        # Get all constraints from the GCS system
        constraints = self.gcs_system.constraints

        for shape1, shape2, constraint_type, value in constraints:
            # Get centers of both shapes
            center1 = self.get_shape_center(shape1)
            center2 = self.get_shape_center(shape2)

            if center1 is None or center2 is None:
                continue

            x1, y1 = center1
            x2, y2 = center2

            # Draw dotted line between shapes
            cr.save()
            cr.set_source_rgb(0.0, 0.5, 1.0)  # Blue color for constraints
            cr.set_line_width(2)
            cr.set_dash([5, 5])  # Dotted line pattern

            cr.move_to(x1, y1)
            cr.line_to(x2, y2)
            cr.stroke()
            cr.restore()

            # Draw label with constraint value at midpoint
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2

            # Only show distance values for distance constraints
            if constraint_type.lower() == "distance":
                cr.save()
                cr.set_source_rgb(0.0, 0.0, 0.0)  # Black text
                cr.select_font_face("Sans", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
                cr.set_font_size(14)

                label = f"{value:.1f}"
                text_extents = cr.text_extents(label)

                # Draw text above the line
                cr.move_to(mid_x - text_extents.width / 2, mid_y - 10)
                cr.show_text(label)
                cr.restore()

    def draw_element_labels(self, widget, cr: cairo.Context):
        """Draw element type labels on shapes"""
        for shape in self.shape_manager.shape_buffer:
            shape_type = self.get_shape_type_name(shape)
            center = self.get_shape_center(shape)

            if center is None:
                continue

            x, y = center

            cr.save()
            cr.set_source_rgb(0.2, 0.2, 0.2)  # Dark gray text
            cr.select_font_face("Sans", cairo.FONT_SLANT_ITALIC, cairo.FONT_WEIGHT_NORMAL)
            cr.set_font_size(12)

            text_extents = cr.text_extents(shape_type)

            # Draw text below the shape center
            cr.move_to(x - text_extents.width / 2, y + 25)
            cr.show_text(shape_type)
            cr.restore()

    def on_draw(self, widget, cr: cairo.Context):

        ### Drawing the grid
        width = widget.get_allocated_width()
        height = widget.get_allocated_height()

        # Grid parameters
        grid_size = 40

        # Set grid line properties
        cr.set_source_rgb(0.8, 0.8, 0.8)  # Light gray
        cr.set_line_width(1)

        # Draw vertical lines
        for x in range(0, width, grid_size):
            cr.move_to(x, 0)
            cr.line_to(x, height)

        # Draw horizontal lines
        for y in range(0, height, grid_size):
            cr.move_to(0, y)
            cr.line_to(width, y)

        cr.stroke()

        ### Drawing the shapes
        for shape in self.shape_manager.shape_buffer:
            shape.on_draw(widget, cr)

        ### Drawing constraints if enabled
        if self.show_constraints and self.gcs_system is not None:
            self.draw_constraints(widget, cr)

        ### Drawing element type labels if constraint rendering is enabled
        if self.show_constraints:
            self.draw_element_labels(widget, cr)

    def on_button_press(self, widget, event):

        if event.type == Gdk.EventType.BUTTON_PRESS:
            if event.button == common.MouseButtonGtkId.LEFT_MOUSE_BUTTON:
                self.handle_draw_events(event)
            elif event.button == common.MouseButtonGtkId.RIGHT_MOUSE_BUTTON:
                self.handle_select_events(event)
            elif event.button == common.MouseButtonGtkId.MIDDLE_MOUSE_BUTTON:
                self.handle_add_constraint_events(event)

    def handle_select_events(self, event):
        # Currently one constraint can be added at a time on the gui
        if len(self.selected_items) < 2:
            for shape in self.shape_manager.shape_buffer:
                if shape.is_hit_by_point(
                    drawmodel.CanvasCoord(event.x, event.y)
                ):
                    # self.selected_items.append({shape, copy.deepcopy(shape.colour)})
                    self.selected_items.append(shape)
                    shape.colour = common.RgbColour(1.0, 0.0, 0.0)
                    self.queue_draw()

    def handle_add_constraint_events(self, event):
        if len(self.selected_items) == 2:
            constraint_dialog = self.ConstraintDialog(
                self.get_toplevel(),
                title="Add Distance Constraint",
                message="Enter distance value:",
            )
            err = constraint_dialog.run()

            if err == Gtk.ResponseType.OK:
                constraint_value = constraint_dialog.get_float()
                if constraint_value is not None and self.gcs_system is not None:
                    # Add constraint to the geometric constraint system
                    success = self.gcs_system.add_constraint_between_shapes(
                        self.selected_items[0],
                        self.selected_items[1],
                        "distance",
                        constraint_value,
                    )
                    if success:
                        print(
                            f"✓ Added distance constraint: {constraint_value}"
                        )
                        # Print current graph info
                        info = self.gcs_system.get_constraint_graph_info()
                        print(
                            f"Graph info: {info['nodes']} nodes, {info['edges']} edges, well-constrained: {info['well_constrained']}"
                        )
                    else:
                        print("✗ Failed to add constraint")
                else:
                    print("✗ Invalid constraint value")

            # Reset selection colors
            for selected in self.selected_items:
                selected.colour = common.RgbColour(
                    0.0, 0.0, 0.0
                )  # Note this deletes the original colour of shape

            self.selected_items.clear()
            constraint_dialog.destroy()
            self.queue_draw()  # Redraw to show updated graph info

    def add_shape_to_constraint_graph(self, shape):
        """Helper method to add a shape to the constraint graph"""
        if self.gcs_system is not None:
            self.gcs_system.add_shape_as_node(shape)

    def handle_draw_events(self, event):
        match self.drawing_method:
            case DrawingMethod.POINT:
                new_point = drawmodel.Point(
                    event.x,
                    event.y,
                    self.current_width,
                    copy.deepcopy(self.current_colour),
                )
                self.shape_manager.shape_buffer.append(new_point)
                # Add to constraint graph
                self.add_shape_to_constraint_graph(new_point)

                self.queue_draw()

            case DrawingMethod.LINE:
                if not self.temp_start_point:
                    self.temp_start_point = True
                    new_point = drawmodel.Point(
                        event.x,
                        event.y,
                        self.current_width,
                        copy.deepcopy(self.current_colour),
                    )
                    self.shape_manager.shape_buffer.append(new_point)
                    # Add to constraint graph
                    self.add_shape_to_constraint_graph(new_point)
                else:
                    end_point = drawmodel.Point(
                        event.x,
                        event.y,
                        copy.deepcopy(self.current_width),
                        copy.deepcopy(self.current_colour),
                    )
                    new_line = drawmodel.Line(
                        self.shape_manager.shape_buffer[-1],
                        end_point,
                        copy.deepcopy(self.current_width),
                        copy.deepcopy(self.current_colour),
                    )
                    self.shape_manager.shape_buffer.append(new_line)
                    self.shape_manager.shape_buffer.append(end_point)

                    # Add both line and end point to constraint graph
                    self.add_shape_to_constraint_graph(new_line)
                    self.add_shape_to_constraint_graph(end_point)

                    self.temp_start_point = False

                self.queue_draw()
            case DrawingMethod.CIRCLE:
                new_circle = drawmodel.Circle(
                    drawmodel.CanvasCoord(event.x, event.y),
                    0.0,
                    self.current_width,
                    copy.deepcopy(self.current_colour),
                )
                self.shape_manager.shape_buffer.append(new_circle)

    def on_motion_notify(self, widget, event):
        match self.drawing_method:
            case DrawingMethod.CIRCLE:
                self.set_circle_radius_and_draw(
                    drawmodel.CanvasCoord(event.x, event.y)
                )

    def on_button_release(self, widget, event):
        if (
            event.type == Gdk.EventType.BUTTON_RELEASE
            and event.button == common.MouseButtonGtkId.LEFT_MOUSE_BUTTON
        ):
            match self.drawing_method:
                case DrawingMethod.CIRCLE:
                    self.set_circle_radius_and_draw(
                        drawmodel.CanvasCoord(event.x, event.y)
                    )
                    # Circle is now complete, add to constraint graph
                    if len(self.shape_manager.shape_buffer) > 0:
                        last_shape = self.shape_manager.shape_buffer[-1]
                        if isinstance(last_shape, drawmodel.Circle):
                            self.add_shape_to_constraint_graph(last_shape)

    def set_circle_radius_and_draw(self, p: drawmodel.CanvasCoord):
        self.shape_manager.shape_buffer[-1].calculate_and_set_radius(
            drawmodel.CanvasCoord(p.x, p.y)
        )
        self.queue_draw()

    def clear_canvas(self):
        self.shape_manager.shape_buffer.clear()
        # Clear the constraint graph as well
        if self.gcs_system is not None:
            self.gcs_system.clear_constraint_graph()
        self.queue_draw()

    def set_drawing_method(self, stringRep: str):
        self.drawing_method = DrawingMethod.get_enum_based_on_str(stringRep)

    def undo_last(self):
        if len(self.shape_manager.shape_buffer) != 0:
            # Get the shape that will be removed
            removed_shape = self.shape_manager.shape_buffer[-1]

            # Remove from shape buffer
            self.shape_manager.shape_buffer.pop()

            # Remove from constraint graph
            if self.gcs_system is not None:
                self.gcs_system.remove_shape(removed_shape)

            self.queue_draw()


# TODO collosion detection
class DrawingLayout(Gtk.Grid):
    def __init__(self, shape_manager: drawmodel.ShapeManager, gcs_system=None):
        super().__init__()
        self.set_column_homogeneous(True)
        self.set_row_homogeneous(True)

        self.drawing_area = DrawingCanvasWidget(shape_manager, gcs_system)

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
            Gtk.Adjustment(
                value=0,
                lower=0,
                upper=1,
                step_increment=0.01,
                page_increment=0.1,
                page_size=0,
            ),
        )
        self.createSpinButton(
            self.on_green_colour_change,
            list_box,
            "Set Green Color",
            Gtk.Adjustment(
                value=0,
                lower=0,
                upper=1,
                step_increment=0.01,
                page_increment=0.1,
                page_size=0,
            ),
        )
        self.createSpinButton(
            self.on_blue_colour_change,
            list_box,
            "Set Blue Color",
            Gtk.Adjustment(
                value=0,
                lower=0,
                upper=1,
                step_increment=0.01,
                page_increment=0.1,
                page_size=0,
            ),
        )

        # Line width setter
        self.createSpinButton(
            self.set_line_width,
            list_box,
            "Set line width",
            Gtk.Adjustment(
                value=5,
                lower=1,
                upper=30,
                step_increment=1,
                page_increment=10,
                page_size=0,
            ),
            0,
        )

        # Add a clear button to the header
        clear_button = Gtk.Button.new_with_label("Clear Canvas")
        clear_button.connect("clicked", self.on_click_clear_draw)
        clear_row = Gtk.ListBoxRow()
        clear_row.add(clear_button)

        list_box.add(clear_row)

        # Add undo button
        undo_button = Gtk.Button.new_with_label("Undo Last")
        undo_button.connect("clicked", self.undo_last)
        undo_row = Gtk.ListBoxRow()
        undo_row.add(undo_button)

        list_box.add(undo_row)

        # Add solve constraints button
        solve_button = Gtk.Button.new_with_label("Solve Constraints")
        solve_button.connect("clicked", self.solve_constraints)
        solve_row = Gtk.ListBoxRow()
        solve_row.add(solve_button)

        list_box.add(solve_row)

        # Add save button
        save_button = Gtk.Button.new_with_label("Save Drawing")
        save_button.connect("clicked", self.on_save_drawing)
        save_row = Gtk.ListBoxRow()
        save_row.add(save_button)

        list_box.add(save_row)

        # Add load button
        load_button = Gtk.Button.new_with_label("Load Drawing")
        load_button.connect("clicked", self.on_load_drawing)
        load_row = Gtk.ListBoxRow()
        load_row.add(load_button)

        list_box.add(load_row)

        # Add constraint rendering toggle switch
        constraint_render_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        constraint_render_label = Gtk.Label(label="Show Constraints")
        self.constraint_render_switch = Gtk.Switch()
        self.constraint_render_switch.set_active(False)
        self.constraint_render_switch.connect("notify::active", self.on_constraint_render_toggle)
        constraint_render_box.pack_start(constraint_render_label, True, True, 0)
        constraint_render_box.pack_start(self.constraint_render_switch, False, False, 0)
        constraint_render_row = Gtk.ListBoxRow()
        constraint_render_row.add(constraint_render_box)

        list_box.add(constraint_render_row)

        self.add(list_box)
        self.attach(self.drawing_area, 1, 0, 5, 1)

    def on_constraint_render_toggle(self, switch, gparam):
        """Handle constraint rendering toggle"""
        is_active = switch.get_active()
        self.drawing_area.show_constraints = is_active
        self.drawing_area.queue_draw()  # Trigger redraw

    def solve_constraints(self, button):
        """Solve the constraint system and update positions"""
        if self.drawing_area.gcs_system is not None:
            print("Attempting to solve constraints...")
            info = self.drawing_area.gcs_system.get_constraint_graph_info()
            print(
                f"Current graph: {info['nodes']} nodes, {info['edges']} edges"
            )

            # Get canvas dimensions to calculate center
            canvas_width = self.drawing_area.get_allocated_width()
            canvas_height = self.drawing_area.get_allocated_height()
            canvas_center_x = canvas_width / 2.0
            canvas_center_y = canvas_height / 2.0

            print(f"Canvas size: {canvas_width}x{canvas_height}, center: ({canvas_center_x:.2f}, {canvas_center_y:.2f})")

            # Call the C++ solver
            success = self.drawing_area.gcs_system.solve_constraint_graph()

            if success:
                print(f"✓ Constraint solving completed")
                # Force redraw of the canvas to show updated positions
                self.drawing_area.queue_draw()
            else:
                print("✗ Constraint solving failed or no constraints to solve")

    def on_save_drawing(self, button):
        """Handle save drawing button click"""
        dialog = Gtk.FileChooserDialog(
            title="Save Drawing",
            parent=self.get_toplevel(),
            action=Gtk.FileChooserAction.SAVE
        )
        dialog.add_buttons(
            Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
            Gtk.STOCK_SAVE, Gtk.ResponseType.OK
        )
        dialog.set_do_overwrite_confirmation(True)
        dialog.set_current_name("drawing.json")

        # Add file filter for JSON files
        filter_json = Gtk.FileFilter()
        filter_json.set_name("JSON files")
        filter_json.add_pattern("*.json")
        dialog.add_filter(filter_json)

        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            filename = dialog.get_filename()
            if not filename.endswith('.json'):
                filename += '.json'

            if self.drawing_area.gcs_system is not None:
                success = self.drawing_area.gcs_system.save_to_file(filename)
                if success:
                    print(f"✓ Drawing saved successfully to {filename}")
                else:
                    print(f"✗ Failed to save drawing")
            else:
                print("✗ No GCS system available")

        dialog.destroy()

    def on_load_drawing(self, button):
        """Handle load drawing button click"""
        dialog = Gtk.FileChooserDialog(
            title="Load Drawing",
            parent=self.get_toplevel(),
            action=Gtk.FileChooserAction.OPEN
        )
        dialog.add_buttons(
            Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
            Gtk.STOCK_OPEN, Gtk.ResponseType.OK
        )

        # Add file filter for JSON files
        filter_json = Gtk.FileFilter()
        filter_json.set_name("JSON files")
        filter_json.add_pattern("*.json")
        dialog.add_filter(filter_json)

        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            filename = dialog.get_filename()

            if self.drawing_area.gcs_system is not None:
                success = self.drawing_area.gcs_system.load_from_file(filename)
                if success:
                    print(f"✓ Drawing loaded successfully from {filename}")
                    # Force redraw of the canvas
                    self.drawing_area.queue_draw()
                else:
                    print(f"✗ Failed to load drawing")
            else:
                print("✗ No GCS system available")

        dialog.destroy()

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

    # These functions could be omitted by directly passing the drawing areas functions
    def on_drawing_method_changed(self, combo):
        self.drawing_area.set_drawing_method(combo.get_active_text())

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

    def undo_last(self, button):
        self.drawing_area.undo_last()
