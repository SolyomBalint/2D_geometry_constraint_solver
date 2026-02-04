#ifndef GUI_MODELLER_VIEW_HPP
#define GUI_MODELLER_VIEW_HPP

// Custom headers
#include "./canvas.hpp"
#include "./constraint_model.hpp"

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief The modeller view containing the canvas, toolbar, and
 *        status bar.
 *
 * Provides tools for creating and editing geometric elements and
 * constraints. All actions are reflected in the underlying
 * ConstraintModel.
 */
class ModellerView : public Gtk::Box {
public:
    explicit ModellerView(ConstraintModel& model);

private:
    void buildToolbar();
    void buildStatusBar();

    void onToolChanged(Tool tool);
    void onConstraintRequested(ElementId elemA, ElementId elemB);
    void onSolveClicked();
    void onStatusChanged(const Glib::ustring& status);

    ConstraintModel& m_model;
    Canvas m_canvas;

    Gtk::Box m_toolbar;
    Gtk::Label m_statusLabel;

    // Tool buttons
    Gtk::ToggleButton m_selectBtn;
    Gtk::ToggleButton m_pointBtn;
    Gtk::ToggleButton m_lineBtn;
    Gtk::ToggleButton m_constraintBtn;
    Gtk::ToggleButton m_deleteBtn;

    // Action buttons
    Gtk::Button m_solveBtn;
};

} // namespace Gui

#endif // GUI_MODELLER_VIEW_HPP
