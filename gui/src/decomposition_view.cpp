#include "decomposition_view.hpp"

// General STD/STL headers
#include <format>
#include <stdexcept>

// Constraint solver headers
#include <geometric_constraint_system.hpp>

namespace Gui {

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
        = Gtk::StringList::create({ "Deficit-based Top-Down Decomposition" });
    m_algorithmDropdown.set_model(m_algorithmModel);
    m_algorithmDropdown.set_selected(0);
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
        m_lastStree.reset();
        return;
    }

    // Make a mutable copy for the decomposition algorithm
    // (getSTreeDecomposition mutates the graph by adding virtual edges)
    auto graphCopy = constGraph;

    try {
        Gcs::DeficitStreeBasedTopDownStrategy strategy;
        auto stree = strategy.getSTreeDecomposition(graphCopy);

        auto leafCount = stree.getLeaves().size();
        auto totalNodes = stree.nodeCount();

        m_lastStree = stree;

        // Populate both canvases
        m_leafCanvas.setDecomposition(stree);
        m_streeCanvas.setDecomposition(stree);

        m_statusLabel.set_text(
            std::format("Decomposed: {} tree nodes, {} leaf components",
                totalNodes, leafCount));

    } catch (const std::exception& e) {
        m_statusLabel.set_text(
            std::format("Decomposition failed: {}", e.what()));
        m_leafCanvas.clear();
        m_streeCanvas.clear();
        m_lastStree.reset();
    }
}

void DecompositionView::onVisualizationChanged()
{
    auto selected = m_vizModeDropdown.get_selected();
    if (selected == 0) {
        m_canvasStack.set_visible_child("leaves");
    } else if (selected == 1) {
        m_canvasStack.set_visible_child("stree");
    }
}

} // namespace Gui
