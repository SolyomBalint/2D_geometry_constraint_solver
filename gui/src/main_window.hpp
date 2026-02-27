#ifndef GUI_MAIN_WINDOW_HPP
#define GUI_MAIN_WINDOW_HPP

// General STD/STL headers
#include <string>

// Custom headers
#include "./constraint_model.hpp"
#include "./decomposition_view.hpp"
#include "./modeller_view.hpp"
#include "./solving_view.hpp"

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief Main application window containing switchable views.
 *
 * Uses a Gtk::Stack with a Gtk::StackSwitcher in the header bar
 * to switch between the Modeller and Decomposition views.
 * Provides Save and Load buttons in the header bar for model
 * persistence.
 */
class MainWindow : public Gtk::ApplicationWindow {
public:
    MainWindow();

private:
    void onSaveClicked();
    void onLoadClicked();

    /**
     * @brief Write serialized JSON to the given file path.
     * @param path The output file path.
     * @param json The JSON content to write.
     */
    void writeFile(const std::string& path, const std::string& json);

    /**
     * @brief Read the contents of a file, deserialize, and load
     *        into the model.
     * @param path The input file path.
     */
    void readAndLoadFile(const std::string& path);

    ConstraintModel m_model;

    Gtk::Stack m_stack;
    Gtk::StackSwitcher m_stackSwitcher;
    Gtk::HeaderBar m_headerBar;

    ModellerView m_modellerView;
    DecompositionView m_decompositionView;
    SolvingView m_solvingView;

    // Save / Load buttons
    Gtk::Button m_saveBtn;
    Gtk::Button m_loadBtn;
};

} // namespace Gui

#endif // GUI_MAIN_WINDOW_HPP
