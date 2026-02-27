#ifndef GUI_MODELLER_VIEW_HPP
#define GUI_MODELLER_VIEW_HPP

// Custom headers
#include "./canvas.hpp"
#include "./constraint_model.hpp"
#include "./model_serializer.hpp"

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

    /** @brief Get the canvas (for serialization). */
    [[nodiscard]] const Canvas& getCanvas() const;

    /** @brief Get the canvas (mutable, for loading). */
    Canvas& getCanvas();

    /**
     * @brief Clear the canvas and rebuild from deserialized data.
     *
     * Clears the canvas visual state, then replays element and
     * constraint creation commands through the model, rebuilding
     * both the constraint graph and the canvas visual records.
     *
     * @param data The deserialized model data to load.
     * @return Empty string on success, or an error description.
     */
    std::string loadModelData(const ModelData& data);

private:
    void buildToolbar();
    void buildStatusBar();

    void onToolChanged(Tool tool);
    void onConstraintRequested(ElementId elemA, ElementId elemB);
    void onAngleConstraintRequested(ElementId elemA, ElementId elemB);
    void onAngleConstraintConfirmed(
        ElementId elemA, ElementId elemB, double angleDegrees, bool flipped);
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
    Gtk::ToggleButton m_distanceConstraintBtn;
    Gtk::ToggleButton m_angleConstraintBtn;
    Gtk::ToggleButton m_deleteBtn;

    // Action buttons
    Gtk::Button m_solveBtn;
};

} // namespace Gui

#endif // GUI_MODELLER_VIEW_HPP
