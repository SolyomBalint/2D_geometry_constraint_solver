#include "decomposition_view.hpp"

// General STD/STL headers
#include <cstddef>
#include <format>
#include <stdexcept>
#include <string>

// Constraint solver headers
#include <decomposition/top_down/stree_top_down_strategy.hpp>

namespace Gui {

namespace {

    constexpr unsigned int TOP_DOWN_ALGORITHM_INDEX = 0;
    [[nodiscard]] bool isTopDownAlgorithmSelected(unsigned int selected)
    {
        return selected == TOP_DOWN_ALGORITHM_INDEX;
    }

} // namespace

DecompositionView::DecompositionView(ConstraintModel& model)
    : Gtk::Box(Gtk::Orientation::VERTICAL)
    , m_model(model)
    , m_toolbar(Gtk::Orientation::HORIZONTAL)
{
    set_hexpand(true);
    set_vexpand(true);

    buildToolbar();
    append(m_toolbar);

    // Set up canvas stack (no visible switcher — controlled by dropdown)
    m_canvasStack.set_hexpand(true);
    m_canvasStack.set_vexpand(true);
    m_canvasStack.set_transition_type(Gtk::StackTransitionType::CROSSFADE);

    m_canvasStack.add(m_leafCanvas, "leaves", "Leaf Components");
    m_canvasStack.add(m_streeCanvas, "stree", "S-Tree");
    m_canvasStack.add(m_drPlanCanvas, "drplan", "DR-Plan");

    append(m_canvasStack);
}

void DecompositionView::buildToolbar()
{
    m_toolbar.set_spacing(8);
    m_toolbar.set_margin(4);
    m_toolbar.add_css_class("toolbar");

    // Algorithm dropdown
    auto algoLabel = Gtk::make_managed<Gtk::Label>("Algorithm:");
    algoLabel->set_margin_start(4);
    m_toolbar.append(*algoLabel);

    m_algorithmModel
        = Gtk::StringList::create({ "Deficit-based Top-Down Decomposition",
            "Bottom-Up DR-Plan Decomposition" });
    m_algorithmDropdown.set_model(m_algorithmModel);
    m_algorithmDropdown.set_selected(0);
    m_algorithmDropdown.property_selected().signal_changed().connect(
        sigc::mem_fun(*this, &DecompositionView::onAlgorithmChanged));
    m_toolbar.append(m_algorithmDropdown);

    // Separator
    auto sep1 = Gtk::make_managed<Gtk::Separator>(Gtk::Orientation::VERTICAL);
    m_toolbar.append(*sep1);

    // Visualization mode dropdown
    auto vizLabel = Gtk::make_managed<Gtk::Label>("View:");
    m_toolbar.append(*vizLabel);

    m_vizModeModel = Gtk::StringList::create({ "Leaf Components", "S-Tree" });
    m_vizModeDropdown.set_model(m_vizModeModel);
    m_vizModeDropdown.set_selected(0);
    m_vizModeDropdown.property_selected().signal_changed().connect(
        sigc::mem_fun(*this, &DecompositionView::onVisualizationChanged));
    m_toolbar.append(m_vizModeDropdown);

    // Separator
    auto sep2 = Gtk::make_managed<Gtk::Separator>(Gtk::Orientation::VERTICAL);
    m_toolbar.append(*sep2);

    // Decompose button
    m_decomposeBtn.set_label("Decompose");
    m_decomposeBtn.add_css_class("suggested-action");
    m_decomposeBtn.signal_clicked().connect(
        sigc::mem_fun(*this, &DecompositionView::onDecompose));
    m_toolbar.append(m_decomposeBtn);

    // Status label
    m_statusLabel.set_hexpand(true);
    m_statusLabel.set_halign(Gtk::Align::END);
    m_statusLabel.set_margin_end(8);
    m_statusLabel.add_css_class("dim-label");
    m_statusLabel.set_text("No decomposition yet");
    m_toolbar.append(m_statusLabel);
}

void DecompositionView::onDecompose()
{
    const auto& constGraph = m_model.getConstraintGraph();

    // Need at least some nodes and edges to decompose
    if (constGraph.nodeCount() < 3) {
        m_statusLabel.set_text("Need at least 3 nodes to decompose");
        m_leafCanvas.clear();
        m_streeCanvas.clear();
        m_drPlanCanvas.clear();
        m_lastStree.reset();
        m_lastBottomUp.reset();
        return;
    }

    const bool topDownSelected
        = isTopDownAlgorithmSelected(m_algorithmDropdown.get_selected());

    try {
        if (topDownSelected) {
            // Make a mutable copy for top-down decomposition.
            auto graphCopy = constGraph;
            Gcs::DeficitStreeBasedTopDownStrategy strategy;
            auto stree = strategy.getSTreeDecomposition(graphCopy);

            auto leafCount = stree.getLeaves().size();
            auto totalNodes = stree.nodeCount();

            m_lastStree = stree;
            m_lastBottomUp.reset();

            m_leafCanvas.setDecomposition(stree);
            m_streeCanvas.setDecomposition(stree);
            m_drPlanCanvas.setEmptyMessage(
                "No DR-plan for top-down decomposition mode.");
            m_drPlanCanvas.clear();

            m_statusLabel.set_text(
                std::format("Top-down: {} tree nodes, {} leaf components",
                    totalNodes, leafCount));
        } else {
            auto reduction = Gcs::reduceBottomUp(constGraph);

            const std::size_t remainingClusterCount
                = reduction.remainingClusters.size();
            const std::size_t rootPlanCount = reduction.rootPlans.size();
            std::size_t totalPlanNodeCount = 0;
            std::size_t edgePrimitiveCount = 0;
            std::size_t trianglePrimitiveCount = 0;
            std::size_t merge3Count = 0;
            std::size_t merge2Count = 0;

            for (const auto& rootPlan : reduction.rootPlans) {
                const auto planNodes = rootPlan.traversePreOrder();
                totalPlanNodeCount += planNodes.size();

                for (const auto& planNodeId : planNodes) {
                    const auto& planNode = rootPlan.getValue(planNodeId);
                    switch (planNode.kind) {
                    case Gcs::PlanNodeKind::EdgePrimitive:
                        ++edgePrimitiveCount;
                        break;
                    case Gcs::PlanNodeKind::TrianglePrimitive:
                        ++trianglePrimitiveCount;
                        break;
                    case Gcs::PlanNodeKind::Merge3:
                        ++merge3Count;
                        break;
                    case Gcs::PlanNodeKind::Merge2:
                        ++merge2Count;
                        break;
                    }
                }
            }

            m_lastBottomUp = reduction;
            m_lastStree.reset();

            if (reduction.rootPlans.empty()) {
                m_drPlanCanvas.setEmptyMessage(
                    "Bottom-up produced no root plan nodes for this model.");
            } else {
                m_drPlanCanvas.setEmptyMessage(
                    "No DR-plan to display. Press Decompose.");
            }

            m_drPlanCanvas.setPlans(reduction.rootPlans, constGraph);
            m_leafCanvas.clear();
            m_streeCanvas.clear();

            std::string statusText
                = std::format("Bottom-up: {} alive clusters, {} root plans, {} "
                              "nodes (E:{} P:{} M3:{} M2:{})",
                    remainingClusterCount, rootPlanCount, totalPlanNodeCount,
                    edgePrimitiveCount, trianglePrimitiveCount, merge3Count,
                    merge2Count);

            if (rootPlanCount > 0 && merge3Count == 0 && merge2Count == 0) {
                statusText
                    += " | WARNING: This DR-plan contains only primitive roots "
                       "(no merge nodes returned by reducer)";
            }

            m_statusLabel.set_text(statusText);
        }

        onVisualizationChanged();

    } catch (const std::exception& e) {
        m_statusLabel.set_text(
            std::format("Decomposition failed: {}", e.what()));
        m_leafCanvas.clear();
        m_streeCanvas.clear();
        m_drPlanCanvas.clear();
        m_lastStree.reset();
        m_lastBottomUp.reset();
    }
}

void DecompositionView::onAlgorithmChanged()
{
    const bool topDownSelected
        = isTopDownAlgorithmSelected(m_algorithmDropdown.get_selected());

    if (topDownSelected) {
        m_vizModeModel
            = Gtk::StringList::create({ "Leaf Components", "S-Tree" });
    } else {
        m_vizModeModel = Gtk::StringList::create({ "DR-Plan" });
    }

    m_vizModeDropdown.set_model(m_vizModeModel);
    m_vizModeDropdown.set_selected(0);
    onVisualizationChanged();
}

void DecompositionView::onVisualizationChanged()
{
    const bool topDownSelected
        = isTopDownAlgorithmSelected(m_algorithmDropdown.get_selected());
    const auto selected = m_vizModeDropdown.get_selected();

    if (!topDownSelected) {
        m_canvasStack.set_visible_child("drplan");
        return;
    }

    if (selected == 0U) {
        m_canvasStack.set_visible_child("leaves");
    } else {
        m_canvasStack.set_visible_child("stree");
    }
}

} // namespace Gui
