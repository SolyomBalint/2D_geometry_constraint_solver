#include "modeller_view.hpp"

// General STD/STL headers
#include <format>
#include <string>

namespace Gui {

ModellerView::ModellerView(ConstraintModel& model)
    : Gtk::Box(Gtk::Orientation::VERTICAL)
    , m_model(model)
    , m_canvas(model)
    , m_toolbar(Gtk::Orientation::HORIZONTAL)
{
    buildToolbar();
    buildStatusBar();

    // Layout: toolbar on top, canvas in the middle, status bar at bottom
    append(m_toolbar);
    append(m_canvas);
    append(m_statusLabel);

    // Connect canvas signals
    m_canvas.signalStatusChanged().connect(
        sigc::mem_fun(*this, &ModellerView::onStatusChanged));
    m_canvas.signalConstraintRequested().connect(
        sigc::mem_fun(*this, &ModellerView::onConstraintRequested));

    // Default tool
    onToolChanged(Tool::Select);
    m_selectBtn.set_active(true);
}

void ModellerView::buildToolbar()
{
    m_toolbar.set_spacing(4);
    m_toolbar.set_margin(4);
    m_toolbar.add_css_class("toolbar");

    m_selectBtn.set_label("Select");
    m_pointBtn.set_label("Point");
    m_lineBtn.set_label("Line");
    m_constraintBtn.set_label("Distance");
    m_deleteBtn.set_label("Delete");

    m_selectBtn.set_tooltip_text("Select and move elements (S)");
    m_pointBtn.set_tooltip_text("Create a point (P)");
    m_lineBtn.set_tooltip_text("Create a line (L)");
    m_constraintBtn.set_tooltip_text(
        "Add distance constraint between elements (D)");
    m_deleteBtn.set_tooltip_text("Delete elements or constraints (X)");

    // Group toggle buttons manually (only one active at a time)
    auto setExclusive = [this](Gtk::ToggleButton& active, Tool tool) {
        if (!active.get_active())
            return;
        if (&active != &m_selectBtn)
            m_selectBtn.set_active(false);
        if (&active != &m_pointBtn)
            m_pointBtn.set_active(false);
        if (&active != &m_lineBtn)
            m_lineBtn.set_active(false);
        if (&active != &m_constraintBtn)
            m_constraintBtn.set_active(false);
        if (&active != &m_deleteBtn)
            m_deleteBtn.set_active(false);
        onToolChanged(tool);
    };

    m_selectBtn.signal_toggled().connect(
        [this, setExclusive]() { setExclusive(m_selectBtn, Tool::Select); });
    m_pointBtn.signal_toggled().connect(
        [this, setExclusive]() { setExclusive(m_pointBtn, Tool::Point); });
    m_lineBtn.signal_toggled().connect(
        [this, setExclusive]() { setExclusive(m_lineBtn, Tool::Line); });
    m_constraintBtn.signal_toggled().connect([this, setExclusive]() {
        setExclusive(m_constraintBtn, Tool::DistanceConstraint);
    });
    m_deleteBtn.signal_toggled().connect(
        [this, setExclusive]() { setExclusive(m_deleteBtn, Tool::Delete); });

    m_toolbar.append(m_selectBtn);
    m_toolbar.append(m_pointBtn);
    m_toolbar.append(m_lineBtn);
    m_toolbar.append(m_constraintBtn);
    m_toolbar.append(m_deleteBtn);

    // Separator before solve button
    auto separator
        = Gtk::make_managed<Gtk::Separator>(Gtk::Orientation::VERTICAL);
    m_toolbar.append(*separator);

    // Solve button
    m_solveBtn.set_label("Solve GCS");
    m_solveBtn.set_tooltip_text("Solve the geometric constraint system");
    m_solveBtn.add_css_class("suggested-action");
    m_solveBtn.signal_clicked().connect(
        sigc::mem_fun(*this, &ModellerView::onSolveClicked));
    m_toolbar.append(m_solveBtn);

    // Separator before info label
    auto infoSeparator
        = Gtk::make_managed<Gtk::Separator>(Gtk::Orientation::VERTICAL);
    m_toolbar.append(*infoSeparator);

    // Info label
    auto infoLabel = Gtk::make_managed<Gtk::Label>(
        "Scroll: zoom | Drag: pan/move | Esc: cancel | Del: delete selected");
    infoLabel->set_hexpand(true);
    infoLabel->set_halign(Gtk::Align::END);
    infoLabel->add_css_class("dim-label");
    m_toolbar.append(*infoLabel);
}

void ModellerView::buildStatusBar()
{
    m_statusLabel.set_halign(Gtk::Align::START);
    m_statusLabel.set_margin(4);
    m_statusLabel.set_text("Ready");
    m_statusLabel.add_css_class("dim-label");
}

void ModellerView::onToolChanged(Tool tool)
{
    m_canvas.setTool(tool);
}

void ModellerView::onSolveClicked()
{
    auto errorMessage = m_model.solveConstraintSystem();

    if (!errorMessage.empty()) {
        m_statusLabel.set_text(Glib::ustring("Solve failed: " + errorMessage));
        return;
    }

    m_canvas.refreshPositionsFromModel();
    m_statusLabel.set_text("Solve completed successfully");
}

void ModellerView::onConstraintRequested(ElementId elemA, ElementId elemB)
{
    // Create a dialog to ask for the distance value
    auto* toplevel = dynamic_cast<Gtk::Window*>(get_root());
    if (!toplevel)
        return;

    auto dialog = Gtk::make_managed<Gtk::Window>();
    dialog->set_title("Distance Constraint");
    dialog->set_transient_for(*toplevel);
    dialog->set_modal(true);
    dialog->set_default_size(300, -1);

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::Orientation::VERTICAL, 12);
    box->set_margin(16);

    auto label = Gtk::make_managed<Gtk::Label>("Enter distance value:");
    label->set_halign(Gtk::Align::START);
    box->append(*label);

    auto entry = Gtk::make_managed<Gtk::Entry>();
    entry->set_placeholder_text("e.g. 100.0");
    entry->set_input_purpose(Gtk::InputPurpose::NUMBER);
    box->append(*entry);

    auto btnBox = Gtk::make_managed<Gtk::Box>(Gtk::Orientation::HORIZONTAL, 8);
    btnBox->set_halign(Gtk::Align::END);

    auto cancelBtn = Gtk::make_managed<Gtk::Button>("Cancel");
    auto okBtn = Gtk::make_managed<Gtk::Button>("OK");
    okBtn->add_css_class("suggested-action");
    btnBox->append(*cancelBtn);
    btnBox->append(*okBtn);
    box->append(*btnBox);

    dialog->set_child(*box);

    cancelBtn->signal_clicked().connect([dialog]() { dialog->close(); });

    okBtn->signal_clicked().connect([this, dialog, entry, elemA, elemB]() {
        auto text = entry->get_text();
        try {
            double distance = std::stod(std::string(text));
            if (distance >= 0.0) {
                auto cid
                    = m_model.addDistanceConstraint(elemA, elemB, distance);
                if (cid.has_value()) {
                    CanvasConstraint cc {};
                    cc.id = *cid;
                    cc.elementA = elemA;
                    cc.elementB = elemB;
                    cc.value = distance;
                    // Access canvas constraints through
                    // non-const reference (we're friends
                    // conceptually)
                    const_cast<
                        std::unordered_map<ConstraintId, CanvasConstraint>&>(
                        m_canvas.getConstraints())[*cid]
                        = cc;
                    m_canvas.queue_draw();
                }
            }
        } catch (...) {
            // Invalid number, ignore
        }
        dialog->close();
    });

    // Handle Enter key in entry
    entry->signal_activate().connect([okBtn]() { okBtn->activate(); });

    dialog->present();
}

void ModellerView::onStatusChanged(const Glib::ustring& status)
{
    m_statusLabel.set_text(status);
}

} // namespace Gui
