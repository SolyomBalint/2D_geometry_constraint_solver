#ifndef GUI_DECOMPOSITION_VIEW_HPP
#define GUI_DECOMPOSITION_VIEW_HPP

// General STD/STL headers
#include <optional>

// Custom headers
#include "./constraint_model.hpp"
#include "./decomposition_canvas.hpp"
#include "./stree_canvas.hpp"

// Constraint solver headers
#include <gcs_data_structures.hpp>
#include <structures/binary_tree.hpp>

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief View for visualizing constraint graph decomposition.
 *
 * Provides a toolbar with:
 * - A decomposition algorithm selector (currently only
 *   "Deficit-based Top-Down Decomposition").
 * - A visualization mode selector (for the deficit-based
 *   strategy: "Leaf Components" or "S-Tree").
 * - A "Decompose" button to trigger decomposition.
 * - A status label showing decomposition results.
 *
 * Below the toolbar, a Gtk::Stack switches between the
 * DecompositionCanvas (leaf view) and STreeCanvas (tree view).
 */
class DecompositionView : public Gtk::Box {
public:
    /**
     * @brief Construct the decomposition view.
     * @param model Reference to the shared ConstraintModel.
     */
    explicit DecompositionView(ConstraintModel& model);

private:
    void buildToolbar();
    void onDecompose();
    void onVisualizationChanged();

    ConstraintModel& m_model;

    // Toolbar widgets
    Gtk::Box m_toolbar;
    Gtk::DropDown m_algorithmDropdown;
    Gtk::DropDown m_vizModeDropdown;
    Gtk::Button m_decomposeBtn;
    Gtk::Label m_statusLabel;

    // String list models for dropdowns
    Glib::RefPtr<Gtk::StringList> m_algorithmModel;
    Glib::RefPtr<Gtk::StringList> m_vizModeModel;

    // Canvas stack
    Gtk::Stack m_canvasStack;
    DecompositionCanvas m_leafCanvas;
    STreeCanvas m_streeCanvas;

    // Stored decomposition result
    std::optional<MathUtils::BinaryTree<Gcs::ConstraintGraph>> m_lastStree;
};

} // namespace Gui

#endif // GUI_DECOMPOSITION_VIEW_HPP
