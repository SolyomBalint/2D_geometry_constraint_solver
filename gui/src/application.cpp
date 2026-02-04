#include "application.hpp"

// Custom headers
#include "./main_window.hpp"

namespace Gui {

Glib::RefPtr<Application> Application::create()
{
    return Glib::make_refptr_for_instance<Application>(new Application());
}

Application::Application()
    : Gtk::Application(
          "org.gcs.constraintsolver", Gio::Application::Flags::DEFAULT_FLAGS)
{
}

void Application::on_startup()
{
    Gtk::Application::on_startup();
    loadCss();
}

void Application::on_activate()
{
    auto window = new MainWindow();
    add_window(*window);
    window->present();

    // Ensure window is deleted when closed
    window->signal_close_request().connect(
        [window]() -> bool {
            delete window;
            return false;
        },
        false);
}

void Application::loadCss()
{
    auto cssProvider = Gtk::CssProvider::create();

    // Minimal CSS for the application
    auto cssData
        = Glib::ustring("window { background-color: @window_bg_color; }"
                        ".toolbar { background-color: @headerbar_bg_color; "
                        "  border-bottom: 1px solid @borders; "
                        "  padding: 4px; }"
                        "drawingarea { background-color: white; }");

    cssProvider->load_from_string(cssData);

    Gtk::StyleContext::add_provider_for_display(Gdk::Display::get_default(),
        cssProvider, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
}

} // namespace Gui
