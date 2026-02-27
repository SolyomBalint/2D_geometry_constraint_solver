#ifndef GUI_SOLVER_PROPERTIES_PANEL_HPP
#define GUI_SOLVER_PROPERTIES_PANEL_HPP

// General STD/STL headers
#include <string>
#include <vector>

// Constraint solver headers
#include <gcs_data_structures.hpp>
#include <solve_result.hpp>

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief Right sidebar panel showing solver step properties.
 *
 * After each solve step, displays:
 * - Component info (step N/M, solver used, status)
 * - Element positions in solver space
 * - Full heuristic analysis (canvas vs solver values, decision)
 *
 * Heuristic values are re-computed GUI-side using the public
 * inline functions from @c solvers/heuristics.hpp.
 */
class SolverPropertiesPanel : public Gtk::ScrolledWindow {
public:
    SolverPropertiesPanel();

    /**
     * @brief Update the panel for a completed solve step.
     *
     * Extracts element positions and computes heuristic values.
     *
     * @param component The solved component.
     * @param result The solve result from classifyAndSolve.
     * @param solverName Name of the solver that was used (must be
     *        determined before solving, since matches() checks
     *        change after element positions are set).
     * @param heuristicType Which heuristic the solver uses:
     *        "triangleOrientation", "lineSignedDistances",
     *        "lineNormalAngle", or "" for unknown.
     * @param stepIndex Zero-based step index.
     * @param totalSteps Total number of subproblems.
     */
    void updateForStep(const Gcs::ConstraintGraph& component,
        const Gcs::SolveResult& result, const std::string& solverName,
        const std::string& heuristicType, int stepIndex, int totalSteps);

    /**
     * @brief Clear the panel to its initial "No step" state.
     */
    void clear();

private:
    /**
     * @brief Build the element positions section.
     * @param component The solved component.
     */
    void buildElementSection(const Gcs::ConstraintGraph& component);

    /**
     * @brief Build the heuristic analysis section.
     * @param component The solved component.
     * @param heuristicType Which heuristic to display.
     */
    void buildHeuristicSection(const Gcs::ConstraintGraph& component,
        const std::string& heuristicType);

    // --- Heuristic builders for each solver type ---
    void buildTriangleOrientationHeuristic(
        const Gcs::ConstraintGraph& component);
    void buildLineSignedDistancesHeuristic(
        const Gcs::ConstraintGraph& component);
    void buildLineNormalAngleHeuristic(const Gcs::ConstraintGraph& component);

    /**
     * @brief Add a labeled section header to the content box.
     */
    void addSectionHeader(const std::string& title);

    /**
     * @brief Add a key-value label pair to the content box.
     */
    void addPropertyRow(const std::string& key, const std::string& value);

    /**
     * @brief Add a plain text label to the content box.
     */
    void addTextRow(const std::string& text);

    Gtk::Box m_contentBox;
};

} // namespace Gui

#endif // GUI_SOLVER_PROPERTIES_PANEL_HPP
