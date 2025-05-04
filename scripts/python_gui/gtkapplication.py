#!/usr/bin/env python3

import sys

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from GcsGui import *
from gi.repository import Gdk, Gio, Gtk


class GeometricConstraintSolverApp(Gtk.Application):

    def __init__(self):
        # Initialize with application ID (must be in reverse domain name notation)
        super().__init__(application_id="com.example.geometricconstraintsolver", flags=Gio.ApplicationFlags.FLAGS_NONE)

        # Connect to application signals
        self.connect("activate", self.on_activate)

    def on_activate(self, app):
        # Check if we already have a window
        win = self.get_windows()
        if win:
            win[0].present()
        else:
            # Load CSS theme
            self.load_css_theme()
            win = MainWindow(self)

    def load_css_theme(self):
        # Create CSS provider
        css_provider = Gtk.CssProvider()

        try:
            # Load CSS from file
            css_provider.load_from_path("./themes/custom.css")

            # Apply the CSS to the application
            screen = Gdk.Screen.get_default()
            style_context = Gtk.StyleContext()
            style_context.add_provider_for_screen(screen, css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
            print("Successfully loaded theme")
        except Exception as e:
            print(f"Error loading CSS theme: {e}")

    def do_startup(self):
        # Chain up to parent implementation
        Gtk.Application.do_startup(self)

        # Create application actions
        quit_action = Gio.SimpleAction.new("quit", None)
        quit_action.connect("activate", lambda action, param: self.quit())
        self.add_action(quit_action)

        self.set_accels_for_action("app.quit", ["<Ctrl>Q"])


def main():
    app = GeometricConstraintSolverApp()
    exit_status = app.run(sys.argv)
    sys.exit(exit_status)


if __name__ == "__main__":
    main()
