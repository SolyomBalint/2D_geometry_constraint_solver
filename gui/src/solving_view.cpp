#include "solving_view.hpp"

// General STD/STL headers
#include <format>
#include <memory>
#include <stdexcept>
#include <unordered_map>

// Constraint solver headers
#include <component_solver.hpp>
#include <constraints.hpp>
#include <elements.hpp>
#include <geometric_constraint_system.hpp>

namespace Gui {

SolvingView::SolvingView(ConstraintModel& model)
    : Gtk::Box(Gtk::Orientation::VERTICAL)
    , m_model(model)
    , m_toolbar(Gtk::Orientation::HORIZONTAL)
    , m_contentBox(Gtk::Orientation::HORIZONTAL)
    , m_splitPane(Gtk::Orientation::HORIZONTAL)
    , m_sidebarSeparator(Gtk::Orientation::VERTICAL)
{
    set_hexpand(true);
    set_vexpand(true);

    buildToolbar();
    append(m_toolbar);

    // Set up the split pane with the two canvases
    m_splitPane.set_hexpand(true);
    m_splitPane.set_vexpand(true);
    m_splitPane.set_wide_handle(true);

    m_splitPane.set_start_child(m_modelCanvas);
    m_splitPane.set_end_child(m_solverCanvas);
    m_splitPane.set_resize_start_child(true);
    m_splitPane.set_resize_end_child(true);
    m_splitPane.set_shrink_start_child(false);
    m_splitPane.set_shrink_end_child(false);

    // Content area: paned + separator + sidebar
    m_contentBox.set_hexpand(true);
    m_contentBox.set_vexpand(true);

    m_splitPane.set_hexpand(true);

    m_contentBox.append(m_splitPane);
    m_contentBox.append(m_sidebarSeparator);
    m_contentBox.append(m_propertiesPanel);

    append(m_contentBox);

    updateButtonStates();
}

void SolvingView::buildToolbar()
{
    m_toolbar.set_spacing(8);
    m_toolbar.set_margin(4);
    m_toolbar.add_css_class("toolbar");

    // Algorithm dropdown
    auto* algoLabel = Gtk::make_managed<Gtk::Label>("Algorithm:");
    algoLabel->set_margin_start(4);
    m_toolbar.append(*algoLabel);

    m_algorithmModel = Gtk::StringList::create({ "Deficit-based Top-Down" });
    m_algorithmDropdown.set_model(m_algorithmModel);
    m_algorithmDropdown.set_selected(0);
    m_toolbar.append(m_algorithmDropdown);

    // Separator
    auto* sep1 = Gtk::make_managed<Gtk::Separator>(Gtk::Orientation::VERTICAL);
    m_toolbar.append(*sep1);

    // Step label
    m_stepLabel.set_text("Step: -/-");
    m_stepLabel.set_margin_start(4);
    m_stepLabel.set_margin_end(4);
    m_toolbar.append(m_stepLabel);

    // Separator
    auto* sep2 = Gtk::make_managed<Gtk::Separator>(Gtk::Orientation::VERTICAL);
    m_toolbar.append(*sep2);

    // Prepare button
    m_prepareBtn.set_label("Prepare");
    m_prepareBtn.add_css_class("suggested-action");
    m_prepareBtn.signal_clicked().connect(
        sigc::mem_fun(*this, &SolvingView::onPrepare));
    m_toolbar.append(m_prepareBtn);

    // Step Back button
    m_stepBackBtn.set_label("\u2190 Back");
    m_stepBackBtn.signal_clicked().connect(
        sigc::mem_fun(*this, &SolvingView::onStepBack));
    m_toolbar.append(m_stepBackBtn);

    // Step button
    m_stepBtn.set_label("Step \u2192");
    m_stepBtn.signal_clicked().connect(
        sigc::mem_fun(*this, &SolvingView::onStep));
    m_toolbar.append(m_stepBtn);

    // Reset button
    m_resetBtn.set_label("Reset");
    m_resetBtn.signal_clicked().connect(
        sigc::mem_fun(*this, &SolvingView::onReset));
    m_toolbar.append(m_resetBtn);

    // Status label
    m_statusLabel.set_hexpand(true);
    m_statusLabel.set_halign(Gtk::Align::END);
    m_statusLabel.set_margin_end(8);
    m_statusLabel.add_css_class("dim-label");
    m_statusLabel.set_text("Ready");
    m_toolbar.append(m_statusLabel);
}

void SolvingView::onPrepare()
{
    const auto& constGraph = m_model.getConstraintGraph();

    if (constGraph.nodeCount() < 3) {
        m_statusLabel.set_text("Need at least 3 nodes to solve");
        return;
    }

    try {
        // Deep-copy the constraint graph
        m_deepCopiedGraph = deepCopyConstraintGraph(constGraph);

        // Snapshot the original model in the left pane
        m_modelCanvas.setGraph(constGraph);

        // Decompose the deep copy
        Gcs::DeficitStreeBasedTopDownStrategy strategy;

        // Check constrainedness first
        auto status = strategy.checkConstraintGraphConstrainedness(
            m_deepCopiedGraph.value());
        if (status != Gcs::Constrainedness::WELL_CONSTRAINED) {
            m_statusLabel.set_text("Graph is not well-constrained");
            m_deepCopiedGraph.reset();
            return;
        }

        m_subproblems
            = strategy.decomposeConstraintGraph(m_deepCopiedGraph.value());

        m_currentStep = 0;

        int totalSteps = static_cast<int>(m_subproblems.size());
        m_stepLabel.set_text(std::format("Step: 0/{}", totalSteps));
        m_statusLabel.set_text(
            std::format("Prepared: {} subproblems", totalSteps));

        // Clear the solver canvas and properties
        m_solverCanvas.clear();
        m_propertiesPanel.clear();

        updateButtonStates();

    } catch (const std::exception& e) {
        m_statusLabel.set_text(std::format("Prepare failed: {}", e.what()));
        m_deepCopiedGraph.reset();
        m_subproblems.clear();
        m_currentStep = -1;
        updateButtonStates();
    }
}

void SolvingView::onStep()
{
    if (m_currentStep < 0
        || m_currentStep >= static_cast<int>(m_subproblems.size())) {
        return;
    }

    auto& component = m_subproblems[static_cast<std::size_t>(m_currentStep)];

    // Classify the solver BEFORE solving (matches() checks
    // numberOfSolvedElements which changes after solving)
    auto [solverName, heuristicType] = classifyComponent(component);

    // Solve this component
    Gcs::SolveResult result = Gcs::classifyAndSolve(component);

    // Add to solver canvas
    m_solverCanvas.addSolvedComponent(&component);

    // Update properties panel
    int totalSteps = static_cast<int>(m_subproblems.size());
    m_propertiesPanel.updateForStep(component, result, solverName,
        heuristicType, m_currentStep, totalSteps);

    // Update step counter
    m_currentStep++;
    m_stepLabel.set_text(std::format("Step: {}/{}", m_currentStep, totalSteps));

    // Update status
    std::string statusStr;
    switch (result.status) {
    case Gcs::SolveStatus::Success:
        statusStr = std::format("Step {} solved successfully", m_currentStep);
        break;
    case Gcs::SolveStatus::Unsupported:
        statusStr
            = std::format("Step {}: unsupported configuration", m_currentStep);
        break;
    case Gcs::SolveStatus::Failed:
        statusStr = std::format("Step {}: solve failed", m_currentStep);
        break;
    }
    m_statusLabel.set_text(statusStr);

    updateButtonStates();
}

void SolvingView::onStepBack()
{
    if (m_currentStep <= 0) {
        return;
    }

    int targetStep = m_currentStep - 1;
    replayToStep(targetStep);
}

void SolvingView::replayToStep(int targetStep)
{
    const auto& constGraph = m_model.getConstraintGraph();

    try {
        // Re-deep-copy and re-decompose from scratch
        m_deepCopiedGraph = deepCopyConstraintGraph(constGraph);

        Gcs::DeficitStreeBasedTopDownStrategy strategy;
        m_subproblems
            = strategy.decomposeConstraintGraph(m_deepCopiedGraph.value());

        // Clear canvas and replay steps
        m_solverCanvas.clear();

        int totalSteps = static_cast<int>(m_subproblems.size());

        // Solve and add all steps up to targetStep
        std::string lastSolverName;
        std::string lastHeuristicType;
        Gcs::SolveResult lastResult = Gcs::SolveResult::success();

        for (int i = 0; i < targetStep; ++i) {
            auto& component = m_subproblems[static_cast<std::size_t>(i)];
            auto [solverName, heuristicType] = classifyComponent(component);
            lastResult = Gcs::classifyAndSolve(component);
            m_solverCanvas.addSolvedComponent(&component);
            lastSolverName = solverName;
            lastHeuristicType = heuristicType;
        }

        m_currentStep = targetStep;

        // Update properties panel for the last solved step
        if (targetStep > 0) {
            auto& lastComponent
                = m_subproblems[static_cast<std::size_t>(targetStep - 1)];
            m_propertiesPanel.updateForStep(lastComponent, lastResult,
                lastSolverName, lastHeuristicType, targetStep - 1, totalSteps);
        } else {
            m_propertiesPanel.clear();
        }

        m_stepLabel.set_text(
            std::format("Step: {}/{}", m_currentStep, totalSteps));
        m_statusLabel.set_text(
            std::format("Stepped back to {}/{}", m_currentStep, totalSteps));

        updateButtonStates();

    } catch (const std::exception& e) {
        m_statusLabel.set_text(std::format("Step back failed: {}", e.what()));
    }
}

void SolvingView::onReset()
{
    m_deepCopiedGraph.reset();
    m_subproblems.clear();
    m_currentStep = -1;

    m_modelCanvas.clear();
    m_solverCanvas.clear();
    m_propertiesPanel.clear();

    m_stepLabel.set_text("Step: -/-");
    m_statusLabel.set_text("Ready");

    updateButtonStates();
}

void SolvingView::updateButtonStates()
{
    bool notPrepared = (m_currentStep < 0);
    bool allDone = !notPrepared
        && m_currentStep >= static_cast<int>(m_subproblems.size());
    bool canStepBack = !notPrepared && m_currentStep > 0;

    m_prepareBtn.set_sensitive(notPrepared);
    m_stepBackBtn.set_sensitive(canStepBack);
    m_stepBtn.set_sensitive(!notPrepared && !allDone);
    m_resetBtn.set_sensitive(!notPrepared);
}

std::pair<std::string, std::string> SolvingView::classifyComponent(
    const Gcs::ConstraintGraph& component)
{
    using namespace Gcs::Solvers;

    if (ZeroFixedPointsTriangleSolver::matches(component)) {
        return { "ZeroFixedPointsTriangleSolver", "triangleOrientation" };
    }
    if (ZeroFixedPPLTriangleSolver::matches(component)) {
        return { "ZeroFixedPPLTriangleSolver", "lineSignedDistances" };
    }
    if (ZeroFixedLLPAngleTriangleSolver::matches(component)) {
        return { "ZeroFixedLLPAngleTriangleSolver", "lineNormalAngle" };
    }
    if (TwoFixedPointsDistanceSolver::matches(component)) {
        return { "TwoFixedPointsDistanceSolver", "triangleOrientation" };
    }
    if (TwoFixedPointsLineSolver::matches(component)) {
        return { "TwoFixedPointsLineSolver", "lineSignedDistances" };
    }
    if (FixedPointAndLineFreePointSolver::matches(component)) {
        return { "FixedPointAndLineFreePointSolver", "triangleOrientation" };
    }
    if (TwoFixedLinesFreePointSolver::matches(component)) {
        return { "TwoFixedLinesFreePointSolver", "triangleOrientation" };
    }
    if (FixedLineAndPointFreeLineSolver::matches(component)) {
        return { "FixedLineAndPointFreeLineSolver", "lineNormalAngle" };
    }

    return { "Unknown", "" };
}

Gcs::ConstraintGraph SolvingView::deepCopyConstraintGraph(
    const Gcs::ConstraintGraph& original)
{
    // Start with a shallow copy (same topology, same
    // shared_ptrs)
    auto copy = original;

    // Build a clone map: old Element* -> new shared_ptr<Element>
    // This preserves sharing within the copy (important for
    // nodes that appear multiple times, though in a single graph
    // each node maps to one element).
    std::unordered_map<Gcs::Element*, std::shared_ptr<Gcs::Element>>
        elementClones;

    const auto& graph = copy.getGraph();

    for (const auto& node : graph.getNodes()) {
        auto oldElem = copy.getElement(node);
        if (!oldElem)
            continue;

        auto it = elementClones.find(oldElem.get());
        if (it == elementClones.end()) {
            auto newElem = std::make_shared<Gcs::Element>(*oldElem);
            elementClones[oldElem.get()] = newElem;
            copy.addElement(node, newElem);
        } else {
            copy.addElement(node, it->second);
        }
    }

    // Clone constraints
    for (const auto& edge : graph.getEdges()) {
        auto oldConstr = copy.getConstraintForEdge(edge);
        if (!oldConstr)
            continue;
        auto newConstr = std::make_shared<Gcs::Constraint>(*oldConstr);
        copy.addConstraint(edge, newConstr);
    }

    return copy;
}

} // namespace Gui
