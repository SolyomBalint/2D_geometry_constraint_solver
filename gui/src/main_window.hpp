#ifndef GUI_MAIN_WINDOW_HPP
#define GUI_MAIN_WINDOW_HPP

// Custom headers
#include "./constraint_model.hpp"
#include "./decomposition_view.hpp"
#include "./modeller_view.hpp"

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief Main application window containing switchable views.
 *
 * Uses a Gtk::Stack with a Gtk::StackSwitcher in the header bar
 * to switch between the Modeller and Decomposition views.
 */
class MainWindow : public Gtk::ApplicationWindow {
public:
    MainWindow();

private:
    ConstraintModel m_model;

    Gtk::Stack m_stack;
    Gtk::StackSwitcher m_stackSwitcher;
    Gtk::HeaderBar m_headerBar;

    ModellerView m_modellerView;
    DecompositionView m_decompositionView;
};

} // namespace Gui

#endif // GUI_MAIN_WINDOW_HPP
