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
    LINE_WITH_POINTS = "Line with Points"
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
            self, parent, shape1, shape2, title="Add Constraint"
        ):
            super().__init__(
                title=title,
                transient_for=parent,
                flags=Gtk.DialogFlags.MODAL
                | Gtk.DialogFlags.DESTROY_WITH_PARENT,
            )
            self.add_button(Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL)
            self.add_button(Gtk.STOCK_OK, Gtk.ResponseType.OK)

            # Store shapes for validation
            self.shape1 = shape1
            self.shape2 = shape2

            # Set default size and border width
            self.set_default_size(350, 200)
            self.set_border_width(10)

            # Create container for content
            content_area = self.get_content_area()

            # Create a vertical box to organize widgets
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
            content_area.add(box)

            # Add info label about selected shapes
            info_label = Gtk.Label()
            info_label.set_markup(
                f"<b>Selected shapes:</b>\n"
                f"Shape 1: {type(shape1).__name__}\n"
                f"Shape 2: {type(shape2).__name__}"
            )
            info_label.set_halign(Gtk.Align.START)
            box.pack_start(info_label, False, False, 0)

            # Add separator
            box.pack_start(Gtk.Separator(), False, False, 0)

            # Add constraint type selector
            type_label = Gtk.Label(label="Constraint Type:")
            type_label.set_halign(Gtk.Align.START)
            box.pack_start(type_label, False, False, 0)

            self.constraint_type_combo = Gtk.ComboBoxText()

            # Determine available constraint types based on selected shapes
            self.available_constraints = self._get_available_constraints()
            for constraint_type in self.available_constraints:
                self.constraint_type_combo.append_text(constraint_type)

            if self.available_constraints:
                self.constraint_type_combo.set_active(0)

            self.constraint_type_combo.connect("changed", self._on_constraint_type_changed)
            box.pack_start(self.constraint_type_combo, False, False, 0)

            # Add value entry (for constraints that need a value)
            self.value_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            self.value_label = Gtk.Label(label="Enter value:")
            self.value_label.set_halign(Gtk.Align.START)
            self.value_box.pack_start(self.value_label, False, False, 0)

            self.entry = Gtk.Entry()
            self.entry.set_activates_default(True)
            self.value_box.pack_start(self.entry, False, False, 0)

            box.pack_start(self.value_box, False, False, 0)

            # Set the OK button as default
            self.set_default_response(Gtk.ResponseType.OK)

            # Update visibility based on initial selection
            self._on_constraint_type_changed(self.constraint_type_combo)

            # Show all widgets
            self.show_all()

        def _get_available_constraints(self):
            """Determine which constraints are available based on selected shapes"""
            constraints = []

            # Distance constraint: available between any two shapes
            constraints.append("Distance")

            # Point-on-Line constraint: only available between Point and Line
            if (isinstance(self.shape1, drawmodel.Point) and isinstance(self.shape2, drawmodel.Line)) or \
               (isinstance(self.shape1, drawmodel.Line) and isinstance(self.shape2, drawmodel.Point)):
                constraints.append("Point on Line")

            # Tangency could be added here for Circle-Circle or Circle-Line
            # constraints.append("Tangency")

            return constraints

        def _on_constraint_type_changed(self, combo):
            """Show/hide value entry based on constraint type"""
            constraint_type = combo.get_active_text()

            if constraint_type == "Point on Line":
                # Point-on-line doesn't need a value
                self.value_box.hide()
            else:
                # Distance and other constraints need a value
                self.value_box.show()
                if constraint_type == "Distance":
                    self.value_label.set_text("Enter distance value:")

        def get_constraint_type(self):
            """Get the selected constraint type"""
            return self.constraint_type_combo.get_active_text()

        def get_float(self):
            """Get the entered float value"""
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
        self.line_with_points_last_point: drawmodel.Point = None  # Track last point in continuous drawing
        self.line_with_points_closing_point: drawmodel.Point = None  # Track point selected for closing the chain
        self.selected_items: list = []
        self.show_constraints: bool = False  # Toggle for constraint rendering

        # Zoom and pan state
        self.zoom_level: float = 1.0  # 1.0 = 100%, 2.0 = 200%, 0.5 = 50%
        self.pan_offset_x: float = 0.0
        self.pan_offset_y: float = 0.0
        self.min_zoom: float = 0.1
        self.max_zoom: float = 10.0

        self.connect("draw", self.on_draw)

        self.set_hexpand(True)
        self.set_vexpand(True)
        self.set_events(
            Gdk.EventMask.BUTTON_PRESS_MASK
            | Gdk.EventMask.BUTTON_RELEASE_MASK
            | Gdk.EventMask.BUTTON1_MOTION_MASK
            | Gdk.EventMask.SCROLL_MASK  # Enable scroll events for zooming
            | Gdk.EventMask.SMOOTH_SCROLL_MASK  # Enable smooth scrolling
            # | Gdk.EventMask.POINTER_MOTION_MASK  # Receives all mouse motion regardless of event
        )  # Mouse events have to be enabled manually
        self.connect("button-press-event", self.on_button_press)
        self.connect("button-release-event", self.on_button_release)
        self.connect("motion-notify-event", self.on_motion_notify)
        self.connect("scroll-event", self.on_scroll)

    def screen_to_world(self, screen_x: float, screen_y: float) -> tuple[float, float]:
        """Convert screen coordinates to world coordinates"""
        world_x = (screen_x - self.pan_offset_x) / self.zoom_level
        world_y = (screen_y - self.pan_offset_y) / self.zoom_level
        return (world_x, world_y)

    def world_to_screen(self, world_x: float, world_y: float) -> tuple[float, float]:
        """Convert world coordinates to screen coordinates"""
        screen_x = world_x * self.zoom_level + self.pan_offset_x
        screen_y = world_y * self.zoom_level + self.pan_offset_y
        return (screen_x, screen_y)

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
        """Draw element type labels with numbers on shapes"""
        # Track count of each shape type
        type_counters = {}

        for shape in self.shape_manager.shape_buffer:
            # Skip invisible shapes (e.g., line defining points)
            if not shape.visible:
                continue

            shape_type = self.get_shape_type_name(shape)
            center = self.get_shape_center(shape)

            if center is None:
                continue

            # Increment counter for this shape type
            if shape_type not in type_counters:
                type_counters[shape_type] = 0
            type_counters[shape_type] += 1

            # Create label with number
            label = f"{shape_type}{type_counters[shape_type]}"

            x, y = center

            cr.save()
            cr.set_source_rgb(0.2, 0.2, 0.2)  # Dark gray text
            cr.select_font_face("Sans", cairo.FONT_SLANT_ITALIC, cairo.FONT_WEIGHT_NORMAL)
            cr.set_font_size(12)

            text_extents = cr.text_extents(label)

            # Draw text below the shape center
            cr.move_to(x - text_extents.width / 2, y + 25)
            cr.show_text(label)
            cr.restore()

    def draw_visual_only_line(self, line: drawmodel.Line, widget, cr: cairo.Context):
        """Draw a visual-only line that stops at the edge of point circles"""
        import math

        x1, y1 = line.defining_point_one.coords.x, line.defining_point_one.coords.y
        x2, y2 = line.defining_point_two.coords.x, line.defining_point_two.coords.y

        # Calculate direction vector
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx * dx + dy * dy)

        if length < 1e-6:  # Degenerate line
            return

        # Normalize direction
        dir_x = dx / length
        dir_y = dy / length

        # Get the radii of the defining points (if they are Point objects)
        radius1 = line.defining_point_one.point_radius if isinstance(line.defining_point_one, drawmodel.Point) else 0
        radius2 = line.defining_point_two.point_radius if isinstance(line.defining_point_two, drawmodel.Point) else 0

        # Shorten the line by the point radii so it stops at the edge
        start_x = x1 + dir_x * radius1
        start_y = y1 + dir_y * radius1
        end_x = x2 - dir_x * radius2
        end_y = y2 - dir_y * radius2

        # Draw the shortened line
        cr.set_line_width(line.line_width)
        cr.set_source_rgb(line.colour.red, line.colour.green, line.colour.blue)
        cr.save()
        cr.move_to(start_x, start_y)
        cr.line_to(end_x, end_y)
        cr.stroke()
        cr.restore()

    def draw_extended_line(self, line: drawmodel.Line, widget, cr: cairo.Context):
        """Draw a line, extending it slightly beyond any points that lie on it"""
        import math

        # Get all visible points
        visible_points = [shape for shape in self.shape_manager.shape_buffer
                         if isinstance(shape, drawmodel.Point) and shape.visible]

        # Line vector and parameters
        x1, y1 = line.defining_point_one.coords.x, line.defining_point_one.coords.y
        x2, y2 = line.defining_point_two.coords.x, line.defining_point_two.coords.y

        dx = x2 - x1
        dy = y2 - y1
        line_length = math.sqrt(dx * dx + dy * dy)

        if line_length < 1e-6:  # Degenerate line
            line.on_draw(widget, cr)
            return

        # Normalize direction vector
        dir_x = dx / line_length
        dir_y = dy / line_length

        # Find points on the line and their projection parameters
        tolerance = line.line_width + 2.0  # Distance tolerance for "on line"
        extension_amount = 10.0  # How much to extend beyond points

        min_t = 0.0
        max_t = line_length

        for point in visible_points:
            px, py = point.coords.x, point.coords.y

            # Calculate perpendicular distance from point to infinite line
            # Using cross product formula
            cross = abs((px - x1) * dy - (py - y1) * dx) / line_length

            if cross <= tolerance:
                # Point is close to the line, calculate projection parameter t
                # t represents position along line: 0 at start, line_length at end
                t = (px - x1) * dir_x + (py - y1) * dir_y

                # Extend the line if this point projects beyond current endpoints
                if t < min_t:
                    min_t = t
                elif t > max_t:
                    max_t = t

        # Calculate new endpoints
        start_x = x1 + dir_x * (min_t - extension_amount)
        start_y = y1 + dir_y * (min_t - extension_amount)
        end_x = x1 + dir_x * (max_t + extension_amount)
        end_y = y1 + dir_y * (max_t + extension_amount)

        # Draw the extended line
        cr.set_line_width(line.line_width)
        cr.set_source_rgb(line.colour.red, line.colour.green, line.colour.blue)
        cr.save()
        cr.move_to(start_x, start_y)
        cr.line_to(end_x, end_y)
        cr.stroke()
        cr.restore()

    def on_draw(self, widget, cr: cairo.Context):

        ### Get canvas dimensions
        width = widget.get_allocated_width()
        height = widget.get_allocated_height()

        ### Apply zoom and pan transformations
        cr.save()
        cr.translate(self.pan_offset_x, self.pan_offset_y)
        cr.scale(self.zoom_level, self.zoom_level)

        ### Drawing the grid (in world space)
        # Grid parameters
        grid_size = 40

        # Calculate visible world space bounds
        # Top-left corner in world space
        world_left = -self.pan_offset_x / self.zoom_level
        world_top = -self.pan_offset_y / self.zoom_level
        # Bottom-right corner in world space
        world_right = (width - self.pan_offset_x) / self.zoom_level
        world_bottom = (height - self.pan_offset_y) / self.zoom_level

        # Extend bounds slightly to ensure grid covers entire canvas
        world_left = int(world_left / grid_size) * grid_size - grid_size
        world_top = int(world_top / grid_size) * grid_size - grid_size
        world_right = int(world_right / grid_size) * grid_size + 2 * grid_size
        world_bottom = int(world_bottom / grid_size) * grid_size + 2 * grid_size

        # Set grid line properties (scale line width inversely with zoom)
        cr.set_source_rgb(0.8, 0.8, 0.8)  # Light gray
        cr.set_line_width(1.0 / self.zoom_level)  # Keep line width constant in screen space

        # Draw vertical lines
        x = world_left
        while x <= world_right:
            cr.move_to(x, world_top)
            cr.line_to(x, world_bottom)
            x += grid_size

        # Draw horizontal lines
        y = world_top
        while y <= world_bottom:
            cr.move_to(world_left, y)
            cr.line_to(world_right, y)
            y += grid_size

        cr.stroke()

        ### Drawing the shapes
        for shape in self.shape_manager.shape_buffer:
            if shape.visible:
                # Special handling for lines
                if isinstance(shape, drawmodel.Line):
                    if shape.add_to_constraint_graph:
                        # Normal lines: extend beyond points for visibility
                        self.draw_extended_line(shape, widget, cr)
                    else:
                        # Visual-only lines: stop at the edge of defining points
                        self.draw_visual_only_line(shape, widget, cr)
                else:
                    shape.on_draw(widget, cr)

        ### Drawing constraints if enabled
        if self.show_constraints and self.gcs_system is not None:
            self.draw_constraints(widget, cr)

        ### Drawing element type labels if constraint rendering is enabled
        if self.show_constraints:
            self.draw_element_labels(widget, cr)

        ### Restore context (undo zoom and pan transformations)
        cr.restore()

    def on_button_press(self, widget, event):

        if event.type == Gdk.EventType.BUTTON_PRESS:
            if event.button == common.MouseButtonGtkId.LEFT_MOUSE_BUTTON:
                self.handle_draw_events(event)
            elif event.button == common.MouseButtonGtkId.RIGHT_MOUSE_BUTTON:
                self.handle_select_events(event)
            elif event.button == common.MouseButtonGtkId.MIDDLE_MOUSE_BUTTON:
                self.handle_add_constraint_events(event)

    def handle_select_events(self, event):
        # Convert screen coordinates to world coordinates
        world_x, world_y = self.screen_to_world(event.x, event.y)

        # Special handling for LINE_WITH_POINTS mode: allow selecting a point to close the chain
        if self.drawing_method == DrawingMethod.LINE_WITH_POINTS:
            # Only allow selecting a point if we have started a chain
            if self.line_with_points_last_point is not None:
                for shape in self.shape_manager.shape_buffer:
                    # Only allow selecting visible points
                    if not isinstance(shape, drawmodel.Point) or not shape.visible:
                        continue
                    if shape.is_hit_by_point(drawmodel.CanvasCoord(world_x, world_y)):
                        # Store the closing point and highlight it
                        if self.line_with_points_closing_point is not None:
                            # Reset previous closing point color
                            self.line_with_points_closing_point.colour = common.RgbColour(0.0, 0.0, 0.0)

                        self.line_with_points_closing_point = shape
                        shape.colour = common.RgbColour(0.0, 1.0, 0.0)  # Green for closing point
                        self.queue_draw()
                        print(f"Selected point for closing chain (next left-click will close)")
                        break
            return

        # Normal constraint selection behavior for other modes
        if len(self.selected_items) < 2:
            for shape in self.shape_manager.shape_buffer:
                # Skip invisible shapes (e.g., line defining points)
                if not shape.visible:
                    continue
                # Skip visual-only lines (they shouldn't be selectable for constraints)
                if isinstance(shape, drawmodel.Line) and not shape.add_to_constraint_graph:
                    continue
                if shape.is_hit_by_point(
                    drawmodel.CanvasCoord(world_x, world_y)
                ):
                    # self.selected_items.append({shape, copy.deepcopy(shape.colour)})
                    self.selected_items.append(shape)
                    shape.colour = common.RgbColour(1.0, 0.0, 0.0)
                    self.queue_draw()

    def handle_add_constraint_events(self, event):
        if len(self.selected_items) == 2:
            constraint_dialog = self.ConstraintDialog(
                self.get_toplevel(),
                self.selected_items[0],
                self.selected_items[1],
            )
            err = constraint_dialog.run()

            if err == Gtk.ResponseType.OK and self.gcs_system is not None:
                constraint_type = constraint_dialog.get_constraint_type()

                if constraint_type == "Distance":
                    constraint_value = constraint_dialog.get_float()
                    if constraint_value is not None:
                        # Add distance constraint
                        success = self.gcs_system.add_constraint_between_shapes(
                            self.selected_items[0],
                            self.selected_items[1],
                            "distance",
                            constraint_value,
                        )
                        if success:
                            print(f"✓ Added distance constraint: {constraint_value}")
                        else:
                            print("✗ Failed to add distance constraint")
                    else:
                        print("✗ Invalid constraint value")

                elif constraint_type == "Point on Line":
                    # Determine which shape is the point and which is the line
                    shape1, shape2 = self.selected_items[0], self.selected_items[1]
                    if isinstance(shape1, drawmodel.Point) and isinstance(shape2, drawmodel.Line):
                        point_shape, line_shape = shape1, shape2
                    elif isinstance(shape1, drawmodel.Line) and isinstance(shape2, drawmodel.Point):
                        line_shape, point_shape = shape1, shape2
                    else:
                        print("✗ Point on Line constraint requires a Point and a Line")
                        point_shape, line_shape = None, None

                    if point_shape and line_shape:
                        # Add point-on-line constraint
                        success = self.gcs_system.add_constraint_between_shapes(
                            point_shape,
                            line_shape,
                            "pointonline",
                            0.0,  # No value needed for point-on-line
                        )
                        if success:
                            print("✓ Added point-on-line constraint")
                        else:
                            print("✗ Failed to add point-on-line constraint")

                # Print current graph info
                info = self.gcs_system.get_constraint_graph_info()
                print(
                    f"Graph info: {info['nodes']} nodes, {info['edges']} edges, "
                    f"well-constrained: {info['well_constrained']}"
                )

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
            # Skip lines marked as visual-only (not to be added to constraint graph)
            if isinstance(shape, drawmodel.Line) and not shape.add_to_constraint_graph:
                return
            self.gcs_system.add_shape_as_node(shape)

    def handle_draw_events(self, event):
        # Convert screen coordinates to world coordinates
        world_x, world_y = self.screen_to_world(event.x, event.y)

        match self.drawing_method:
            case DrawingMethod.POINT:
                new_point = drawmodel.Point(
                    world_x,
                    world_y,
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
                        world_x,
                        world_y,
                        self.current_width,
                        copy.deepcopy(self.current_colour),
                        visible=False,  # Hide line defining points
                    )
                    self.shape_manager.shape_buffer.append(new_point)
                    # Don't add defining point to constraint graph
                else:
                    end_point = drawmodel.Point(
                        world_x,
                        world_y,
                        copy.deepcopy(self.current_width),
                        copy.deepcopy(self.current_colour),
                        visible=False,  # Hide line defining points
                    )
                    new_line = drawmodel.Line(
                        self.shape_manager.shape_buffer[-1],
                        end_point,
                        copy.deepcopy(self.current_width),
                        copy.deepcopy(self.current_colour),
                    )
                    self.shape_manager.shape_buffer.append(new_line)
                    self.shape_manager.shape_buffer.append(end_point)

                    # Add only the line to constraint graph (not the defining points)
                    self.add_shape_to_constraint_graph(new_line)

                    self.temp_start_point = False

                self.queue_draw()

            case DrawingMethod.LINE_WITH_POINTS:
                # Check if we're in closing mode
                if self.line_with_points_closing_point is not None:
                    # Close the chain by connecting last point to closing point
                    closing_line = drawmodel.Line(
                        self.line_with_points_last_point,
                        self.line_with_points_closing_point,
                        copy.deepcopy(self.current_width),
                        copy.deepcopy(self.current_colour),
                        visible=True,
                        add_to_constraint_graph=False  # Visual-only line
                    )
                    self.shape_manager.shape_buffer.append(closing_line)

                    # Reset closing point color
                    self.line_with_points_closing_point.colour = common.RgbColour(0.0, 0.0, 0.0)

                    # Reset state to prepare for new chain
                    self.line_with_points_last_point = None
                    self.line_with_points_closing_point = None

                    print("Chain closed. Next click will start a new chain.")
                    self.queue_draw()
                else:
                    # Normal continuous point drawing mode
                    new_point = drawmodel.Point(
                        world_x,
                        world_y,
                        self.current_width,
                        copy.deepcopy(self.current_colour),
                        visible=True,  # Show points
                    )

                    if self.line_with_points_last_point is None:
                        # First point in the chain
                        self.shape_manager.shape_buffer.append(new_point)
                        self.add_shape_to_constraint_graph(new_point)
                        self.line_with_points_last_point = new_point
                    else:
                        # Subsequent points - connect to previous point with a visual-only line
                        connecting_line = drawmodel.Line(
                            self.line_with_points_last_point,
                            new_point,
                            copy.deepcopy(self.current_width),
                            copy.deepcopy(self.current_colour),
                            visible=True,
                            add_to_constraint_graph=False  # Visual-only line
                        )
                        self.shape_manager.shape_buffer.append(connecting_line)
                        self.shape_manager.shape_buffer.append(new_point)

                        # Add only the new point to constraint graph (not the line)
                        self.add_shape_to_constraint_graph(new_point)

                        # Update last point to the new point for next iteration
                        self.line_with_points_last_point = new_point

                    self.queue_draw()

            case DrawingMethod.CIRCLE:
                new_circle = drawmodel.Circle(
                    drawmodel.CanvasCoord(world_x, world_y),
                    0.0,
                    self.current_width,
                    copy.deepcopy(self.current_colour),
                )
                self.shape_manager.shape_buffer.append(new_circle)

    def on_motion_notify(self, widget, event):
        # Convert screen coordinates to world coordinates
        world_x, world_y = self.screen_to_world(event.x, event.y)

        match self.drawing_method:
            case DrawingMethod.CIRCLE:
                self.set_circle_radius_and_draw(
                    drawmodel.CanvasCoord(world_x, world_y)
                )

    def on_scroll(self, widget, event):
        """Handle mouse scroll for zooming"""
        # Determine zoom direction
        zoom_factor = 1.1  # 10% zoom per scroll step

        if event.direction == Gdk.ScrollDirection.UP:
            # Zoom in
            new_zoom = self.zoom_level * zoom_factor
        elif event.direction == Gdk.ScrollDirection.DOWN:
            # Zoom out
            new_zoom = self.zoom_level / zoom_factor
        elif event.direction == Gdk.ScrollDirection.SMOOTH:
            # Smooth scrolling (touchpad)
            success, delta_x, delta_y = event.get_scroll_deltas()
            if success:
                # delta_y < 0 means scroll up (zoom in), > 0 means scroll down (zoom out)
                zoom_change = 1.0 - (delta_y * 0.05)  # 5% per delta unit
                new_zoom = self.zoom_level * zoom_change
            else:
                return False
        else:
            return False

        # Clamp zoom level
        new_zoom = max(self.min_zoom, min(self.max_zoom, new_zoom))

        if new_zoom == self.zoom_level:
            return False  # No change

        # Zoom to mouse cursor position
        # Convert mouse position to world coordinates before zoom
        mouse_x = event.x
        mouse_y = event.y
        world_x_before = (mouse_x - self.pan_offset_x) / self.zoom_level
        world_y_before = (mouse_y - self.pan_offset_y) / self.zoom_level

        # Update zoom
        self.zoom_level = new_zoom

        # Calculate new screen position of the same world point
        screen_x_after = world_x_before * self.zoom_level + self.pan_offset_x
        screen_y_after = world_y_before * self.zoom_level + self.pan_offset_y

        # Adjust pan offset to keep the point under the mouse
        self.pan_offset_x += mouse_x - screen_x_after
        self.pan_offset_y += mouse_y - screen_y_after

        # Trigger redraw
        self.queue_draw()
        return True

    def on_button_release(self, widget, event):
        if (
            event.type == Gdk.EventType.BUTTON_RELEASE
            and event.button == common.MouseButtonGtkId.LEFT_MOUSE_BUTTON
        ):
            # Convert screen coordinates to world coordinates
            world_x, world_y = self.screen_to_world(event.x, event.y)

            match self.drawing_method:
                case DrawingMethod.CIRCLE:
                    self.set_circle_radius_and_draw(
                        drawmodel.CanvasCoord(world_x, world_y)
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
        # Reset continuous drawing state when changing modes
        if self.line_with_points_closing_point is not None:
            # Reset color of closing point if it was set
            self.line_with_points_closing_point.colour = common.RgbColour(0.0, 0.0, 0.0)
        self.line_with_points_last_point = None
        self.line_with_points_closing_point = None

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
            DrawingMethod.LINE_WITH_POINTS,
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

        # Add demo selector label
        demo_label = Gtk.Label(label="Load Demo:")
        demo_label.set_halign(Gtk.Align.START)
        demo_label_row = Gtk.ListBoxRow()
        demo_label_row.add(demo_label)
        list_box.add(demo_label_row)

        # Add demo selector ComboBox
        self.demo_combo = Gtk.ComboBoxText()
        self.demo_combo.set_entry_text_column(0)
        self.demo_combo.connect("changed", self.on_demo_selected)
        demo_combo_row = Gtk.ListBoxRow()
        demo_combo_row.add(self.demo_combo)
        list_box.add(demo_combo_row)

        # Populate demo combo box with available demos
        self.populate_demo_list()

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

    def populate_demo_list(self):
        """Populate the demo selector with JSON files from the demos folder"""
        import os

        # Get the project root and demos directory path
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
        demos_dir = os.path.join(project_root, 'scripts/python_gui/demos')

        # Clear existing items
        self.demo_combo.remove_all()

        # Add a placeholder option
        self.demo_combo.append_text("-- Select a demo --")

        # Check if demos directory exists
        if not os.path.exists(demos_dir):
            print(f"Demos directory not found: {demos_dir}")
            self.demo_combo.set_active(0)
            return

        # Find all JSON files in the demos directory
        try:
            json_files = [f for f in os.listdir(demos_dir) if f.endswith('.json')]
            json_files.sort()  # Sort alphabetically

            if not json_files:
                print(f"No JSON demo files found in {demos_dir}")
            else:
                for json_file in json_files:
                    # Add the filename (without path) to the combo box
                    self.demo_combo.append_text(json_file)
                print(f"Found {len(json_files)} demo file(s)")

        except Exception as e:
            print(f"Error reading demos directory: {e}")

        # Set the placeholder as active
        self.demo_combo.set_active(0)

    def on_demo_selected(self, combo):
        """Handle demo selection from the combo box"""
        import os

        selected_demo = combo.get_active_text()

        # Ignore the placeholder selection
        if selected_demo is None or selected_demo == "-- Select a demo --":
            return

        # Get the full path to the demo file
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
        demos_dir = os.path.join(project_root, 'scripts/python_gui/demos')
        demo_path = os.path.join(demos_dir, selected_demo)

        # Load the demo file
        if self.drawing_area.gcs_system is not None:
            print(f"Loading demo: {selected_demo}")
            success = self.drawing_area.gcs_system.load_from_file(demo_path)
            if success:
                print(f"✓ Demo loaded successfully: {selected_demo}")
                # Force redraw of the canvas
                self.drawing_area.queue_draw()
            else:
                print(f"✗ Failed to load demo: {selected_demo}")
        else:
            print("✗ No GCS system available")

        # Reset the combo box to placeholder
        combo.set_active(0)
