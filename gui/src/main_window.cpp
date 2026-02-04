#include "main_window.hpp"

namespace Gui {

MainWindow::MainWindow()
    : m_modellerView(m_model)
    , m_decompositionView(m_model)
{
    set_title("2D Geometry Constraint Solver");
    set_default_size(1280, 800);

    // Header bar with view switcher
    m_stackSwitcher.set_stack(m_stack);
    m_headerBar.set_title_widget(m_stackSwitcher);
    set_titlebar(m_headerBar);

    // Add views to the stack
    m_stack.add(m_modellerView, "modeller", "Modeller");
    m_stack.add(m_decompositionView, "decomposition", "Decomposition View");

    m_stack.set_transition_type(Gtk::StackTransitionType::SLIDE_LEFT_RIGHT);

    set_child(m_stack);
}

} // namespace Gui
