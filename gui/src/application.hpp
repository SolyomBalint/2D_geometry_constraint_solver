#ifndef GUI_APPLICATION_HPP
#define GUI_APPLICATION_HPP

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief GTK4 application class for the constraint solver GUI.
 *
 * Manages the application lifecycle, CSS styling, and window creation.
 */
class Application : public Gtk::Application {
public:
    static Glib::RefPtr<Application> create();

protected:
    Application();

    void on_activate() override;
    void on_startup() override;

private:
    void loadCss();
};

} // namespace Gui

#endif // GUI_APPLICATION_HPP
