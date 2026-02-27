#ifndef GUI_SOLVING_VIEW_HPP
#define GUI_SOLVING_VIEW_HPP

// General STD/STL headers
#include <optional>
#include <string>
#include <utility>
#include <vector>

// Custom headers
#include "./constraint_model.hpp"
#include "./model_static_canvas.hpp"
#include "./solver_properties_panel.hpp"
#include "./solver_step_canvas.hpp"

// Constraint solver headers
#include <gcs_data_structures.hpp>

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief View for step-by-step visualization of the constraint
 *        solver.
 *
 * Provides:
 * - A toolbar with algorithm selector, step controls
 *   (Prepare / Step / Reset), and status display.
 * - A split pane with:
 *   - Left: static snapshot of the original model (canvas space).
 *   - Right: progressive reconstruction (solver space).
 * - A right sidebar showing solver properties and heuristic
 *   analysis for each step.
 *
 * The solver operates on a deep copy of the constraint graph so
 * that the original model is never modified.
 */
class SolvingView : public Gtk::Box {
public:
    /**
     * @brief Construct the solving view.
     * @param model Reference to the shared ConstraintModel.
     */
    explicit SolvingView(ConstraintModel& model);

private:
    void buildToolbar();
    void onPrepare();
    void onStep();
    void onStepBack();
    void onReset();
    void updateButtonStates();

    /**
     * @brief Re-solve from scratch up to a given step.
     *
     * Deep-copies the original graph, decomposes it, and
     * replays all solve steps from 0 to @p targetStep - 1.
     * Used by onStepBack() to undo the last step.
     *
     * @param targetStep The step to stop at (exclusive).
     */
    void replayToStep(int targetStep);

    /**
     * @brief Classify which solver matches a component and which
     *        heuristic it uses.
     *
     * Must be called BEFORE solving, since matches() checks
     * numberOfSolvedElements() which changes after solving.
     *
     * @param component The component to classify.
     * @return Pair of (solverName, heuristicType).
     */
    static std::pair<std::string, std::string> classifyComponent(
        const Gcs::ConstraintGraph& component);

    /**
     * @brief Create a deep copy of a ConstraintGraph.
     *
     * Clones all Element and Constraint shared_ptr objects so
     * that the copy is fully independent of the original.
     * The topology (nodes, edges, virtual edges) is preserved
     * with the same IDs.
     *
     * @param original The graph to deep-copy.
     * @return A fully independent deep copy.
     */
    static Gcs::ConstraintGraph deepCopyConstraintGraph(
        const Gcs::ConstraintGraph& original);

    ConstraintModel& m_model;

    // Toolbar widgets
    Gtk::Box m_toolbar;
    Gtk::DropDown m_algorithmDropdown;
    Glib::RefPtr<Gtk::StringList> m_algorithmModel;
    Gtk::Button m_prepareBtn;
    Gtk::Button m_stepBackBtn;
    Gtk::Button m_stepBtn;
    Gtk::Button m_resetBtn;
    Gtk::Label m_stepLabel;
    Gtk::Label m_statusLabel;

    // Main content area: canvases + sidebar
    Gtk::Box m_contentBox; ///< Horizontal box: paned + sidebar
    Gtk::Paned m_splitPane;
    ModelStaticCanvas m_modelCanvas;
    SolverStepCanvas m_solverCanvas;
    SolverPropertiesPanel m_propertiesPanel;

    Gtk::Separator m_sidebarSeparator;

    // Solving state
    std::optional<Gcs::ConstraintGraph> m_deepCopiedGraph;
    std::vector<Gcs::ConstraintGraph> m_subproblems;
    int m_currentStep = -1; ///< -1 = not prepared
};

} // namespace Gui

#endif // GUI_SOLVING_VIEW_HPP
