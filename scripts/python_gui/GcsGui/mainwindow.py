import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gio, Gtk

from .model import drawmodel, gcs
from .view import drawmodelview, gcsview


class MainWindow(Gtk.ApplicationWindow):

    def __init__(self, application):
        # Connect to the application
        super().__init__(
            application=application, title="Geometric Constraint Solver"
        )

        self.set_default_size(1920, 1200)

        # Set up the headerbar
        self.header = Gtk.HeaderBar()
        self.header.set_show_close_button(True)
        self.header.props.title = "Geometric Constraint Solver"
        self.set_titlebar(self.header)

        # Add a menu button to the header
        menu_button = Gtk.Button.new_from_icon_name(
            "open-menu-symbolic", Gtk.IconSize.BUTTON
        )
        menu_button.connect("clicked", self.on_menu_clicked)
        self.header.pack_end(menu_button)

        # Create grid layout
        self.grid = Gtk.Grid()
        self.grid.set_column_homogeneous(True)
        self.grid.set_row_homogeneous(True)
        self.add(self.grid)

        stack = Gtk.Stack()
        stack.set_transition_type(Gtk.StackTransitionType.SLIDE_LEFT_RIGHT)
        stack.set_transition_duration(500)

        # Initialize the data containers. They should now about each other for some usecases
        # Note that it is not good practice to bind the lifetime of these objects to the lifetime of the MainWindow
        # But it is sufficient for our demo goals.
        self.shape_data = drawmodel.ShapeManager()
        self.gcs_data = gcs.GeometricConstraintSystem(self.shape_data)

        # Create drawing area
        drawing_area = drawmodelview.DrawingLayout(
            self.shape_data, self.gcs_data
        )

        # Create graph rendering
        graph_render = gcsview.create_constraint_graph_view(self.gcs_data)

        stack.add_titled(drawing_area, "Draw Model", "Draw Model")
        stack.add_titled(graph_render, "Graph Rendering", "Graph Rendering")
        self.grid.add(stack)

        stack_switcher = Gtk.StackSwitcher()
        stack_switcher.set_stack(stack)
        self.header.pack_start(stack_switcher)

        # Connect destroy signal for cleanup
        self.connect("destroy", self.on_window_destroy)

        # Show all widgets
        self.show_all()

    def on_window_destroy(self, widget):
        """Cleanup resources when window is destroyed"""
        if hasattr(self, 'gcs_data') and self.gcs_data is not None:
            self.gcs_data.cleanup()
        if hasattr(self, 'shape_data') and self.shape_data is not None:
            self.shape_data.shape_buffer.clear()

    def on_clear_clicked(self, button):
        self.drawing_area.clear_canvas()

    def on_menu_clicked(self, button):
        # Create a popover menu
        popover = Gtk.Popover()
        popover.set_relative_to(button)

        # Create a box for the menu items
        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        vbox.set_border_width(5)

        # Add menu items
        about_button = Gtk.ModelButton()
        about_button.set_label("About")
        about_button.connect("clicked", self.on_about_clicked)

        quit_button = Gtk.ModelButton()
        quit_button.set_label("Quit")
        quit_button.connect("clicked", lambda _: self.get_application().quit())

        vbox.pack_start(about_button, False, False, 0)
        vbox.pack_start(Gtk.Separator(), False, False, 0)
        vbox.pack_start(quit_button, False, False, 0)

        # Add the box to the popover and show it
        popover.add(vbox)
        popover.set_position(Gtk.PositionType.BOTTOM)
        vbox.show_all()
        popover.popup()

    def on_about_clicked(self, button):
        about_dialog = Gtk.AboutDialog(transient_for=self)
        about_dialog.set_program_name("Geometric Constraint Solver")
        about_dialog.set_version("0.0")
        about_dialog.set_authors(["Bálint Sólyom", "Dr. Vatikus Márton"])
        about_dialog.set_comments(
            "A tool for solving geometric constraint systems"
        )
        about_dialog.set_copyright("© 2025")
        about_dialog.run()
        about_dialog.destroy()
