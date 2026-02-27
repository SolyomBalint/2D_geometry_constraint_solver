#include "modeller_view.hpp"

// General STD/STL headers
#include <format>
#include <string>
#include <unordered_map>
#include <vector>

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
    m_canvas.signalAngleConstraintRequested().connect(
        sigc::mem_fun(*this, &ModellerView::onAngleConstraintRequested));
    m_canvas.signalAngleConstraintConfirmed().connect(
        sigc::mem_fun(*this, &ModellerView::onAngleConstraintConfirmed));

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
    m_distanceConstraintBtn.set_label("Distance");
    m_angleConstraintBtn.set_label("Angle");
    m_deleteBtn.set_label("Delete");

    m_selectBtn.set_tooltip_text("Select and move elements (S)");
    m_pointBtn.set_tooltip_text("Create a point (P)");
    m_lineBtn.set_tooltip_text("Create a line (L)");
    m_distanceConstraintBtn.set_tooltip_text(
        "Add distance constraint between elements (D)");
    m_angleConstraintBtn.set_tooltip_text(
        "Add angle constraint between two lines (A)");
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
        if (&active != &m_distanceConstraintBtn)
            m_distanceConstraintBtn.set_active(false);
        if (&active != &m_angleConstraintBtn)
            m_angleConstraintBtn.set_active(false);
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
    m_distanceConstraintBtn.signal_toggled().connect([this, setExclusive]() {
        setExclusive(m_distanceConstraintBtn, Tool::DistanceConstraint);
    });
    m_angleConstraintBtn.signal_toggled().connect([this, setExclusive]() {
        setExclusive(m_angleConstraintBtn, Tool::AngleConstraint);
    });
    m_deleteBtn.signal_toggled().connect(
        [this, setExclusive]() { setExclusive(m_deleteBtn, Tool::Delete); });

    m_toolbar.append(m_selectBtn);
    m_toolbar.append(m_pointBtn);
    m_toolbar.append(m_lineBtn);
    m_toolbar.append(m_distanceConstraintBtn);
    m_toolbar.append(m_angleConstraintBtn);
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

    auto buttonBox
        = Gtk::make_managed<Gtk::Box>(Gtk::Orientation::HORIZONTAL, 8);
    buttonBox->set_halign(Gtk::Align::END);

    auto cancelBtn = Gtk::make_managed<Gtk::Button>("Cancel");
    auto okBtn = Gtk::make_managed<Gtk::Button>("OK");
    okBtn->add_css_class("suggested-action");
    buttonBox->append(*cancelBtn);
    buttonBox->append(*okBtn);
    box->append(*buttonBox);

    dialog->set_child(*box);

    cancelBtn->signal_clicked().connect([dialog]() { dialog->close(); });

    okBtn->signal_clicked().connect([this, dialog, entry, elemA, elemB]() {
        auto text = entry->get_text();
        try {
            double distance = std::stod(std::string(text));
            if (distance >= 0.0) {
                auto constraintId
                    = m_model.addDistanceConstraint(elemA, elemB, distance);
                if (constraintId.has_value()) {
                    CanvasConstraint canvasConstraint {};
                    canvasConstraint.id = *constraintId;
                    canvasConstraint.elementA = elemA;
                    canvasConstraint.elementB = elemB;
                    canvasConstraint.value = distance;
                    canvasConstraint.type = CanvasConstraintType::Distance;
                    m_canvas.addCanvasConstraint(canvasConstraint);
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

void ModellerView::onAngleConstraintRequested(ElementId elemA, ElementId elemB)
{
    // Create a dialog to ask for the angle value in degrees
    auto* toplevel = dynamic_cast<Gtk::Window*>(get_root());
    if (!toplevel)
        return;

    auto dialog = Gtk::make_managed<Gtk::Window>();
    dialog->set_title("Angle Constraint");
    dialog->set_transient_for(*toplevel);
    dialog->set_modal(true);
    dialog->set_default_size(300, -1);

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::Orientation::VERTICAL, 12);
    box->set_margin(16);

    auto label = Gtk::make_managed<Gtk::Label>("Enter angle value (degrees):");
    label->set_halign(Gtk::Align::START);
    box->append(*label);

    auto entry = Gtk::make_managed<Gtk::Entry>();
    entry->set_placeholder_text("e.g. 45.0");
    entry->set_input_purpose(Gtk::InputPurpose::NUMBER);
    box->append(*entry);

    auto buttonBox
        = Gtk::make_managed<Gtk::Box>(Gtk::Orientation::HORIZONTAL, 8);
    buttonBox->set_halign(Gtk::Align::END);

    auto cancelBtn = Gtk::make_managed<Gtk::Button>("Cancel");
    auto okBtn = Gtk::make_managed<Gtk::Button>("OK");
    okBtn->add_css_class("suggested-action");
    buttonBox->append(*cancelBtn);
    buttonBox->append(*okBtn);
    box->append(*buttonBox);

    dialog->set_child(*box);

    cancelBtn->signal_clicked().connect([dialog]() { dialog->close(); });

    okBtn->signal_clicked().connect([this, dialog, entry, elemA, elemB]() {
        auto text = entry->get_text();
        try {
            double angleDegrees = std::stod(std::string(text));
            if (angleDegrees > 0.0 && angleDegrees < 360.0) {
                // Enter placement mode: preview the arc and let the
                // user click on which side to place it.
                m_canvas.startAnglePlacement(elemA, elemB, angleDegrees);
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

void ModellerView::onAngleConstraintConfirmed(
    ElementId elemA, ElementId elemB, double angleDegrees, bool flipped)
{
    auto constraintId
        = m_model.addAngleConstraint(elemA, elemB, angleDegrees, flipped);
    if (constraintId.has_value()) {
        CanvasConstraint canvasConstraint {};
        canvasConstraint.id = *constraintId;
        canvasConstraint.elementA = elemA;
        canvasConstraint.elementB = elemB;
        canvasConstraint.value = angleDegrees;
        canvasConstraint.type = CanvasConstraintType::Angle;
        canvasConstraint.flipped = flipped;
        m_canvas.addCanvasConstraint(canvasConstraint);
    }
}

void ModellerView::onStatusChanged(const Glib::ustring& status)
{
    m_statusLabel.set_text(status);
}

const Canvas& ModellerView::getCanvas() const
{
    return m_canvas;
}

Canvas& ModellerView::getCanvas()
{
    return m_canvas;
}

std::string ModellerView::loadModelData(const ModelData& data)
{
    m_canvas.clearAll();

    // Maps element array index -> newly assigned ElementId.
    std::vector<ElementId> indexToId;
    indexToId.reserve(data.elements.size());

    for (const auto& elem : data.elements) {
        ElementId newId = 0;
        if (elem.type == CanvasElementType::Point) {
            newId = m_model.addPoint(elem.x, elem.y);
        } else {
            newId = m_model.addLine(elem.x, elem.y, elem.x2, elem.y2);
        }

        CanvasElement canvasElem {};
        canvasElem.id = newId;
        canvasElem.type = elem.type;
        canvasElem.x = elem.x;
        canvasElem.y = elem.y;
        canvasElem.x2 = elem.x2;
        canvasElem.y2 = elem.y2;
        m_canvas.addCanvasElement(canvasElem);

        indexToId.push_back(newId);
    }

    for (const auto& constr : data.constraints) {
        if (constr.elementA >= indexToId.size()
            || constr.elementB >= indexToId.size()) {
            return std::format("Constraint references invalid element index "
                               "({} or {}; {} elements loaded)",
                constr.elementA, constr.elementB, indexToId.size());
        }

        ElementId elemA = indexToId[constr.elementA];
        ElementId elemB = indexToId[constr.elementB];

        if (constr.type == CanvasConstraintType::Distance) {
            auto cid
                = m_model.addDistanceConstraint(elemA, elemB, constr.value);
            if (!cid.has_value()) {
                return std::format("Failed to add distance constraint between "
                                   "elements {} and {}",
                    constr.elementA, constr.elementB);
            }

            CanvasConstraint cc {};
            cc.id = *cid;
            cc.elementA = elemA;
            cc.elementB = elemB;
            cc.value = constr.value;
            cc.type = CanvasConstraintType::Distance;
            m_canvas.addCanvasConstraint(cc);

        } else {
            auto cid = m_model.addAngleConstraint(
                elemA, elemB, constr.value, constr.flipped);
            if (!cid.has_value()) {
                return std::format("Failed to add angle constraint between "
                                   "elements {} and {}",
                    constr.elementA, constr.elementB);
            }

            CanvasConstraint cc {};
            cc.id = *cid;
            cc.elementA = elemA;
            cc.elementB = elemB;
            cc.value = constr.value;
            cc.type = CanvasConstraintType::Angle;
            cc.flipped = constr.flipped;
            m_canvas.addCanvasConstraint(cc);
        }
    }

    // Restore viewport.
    m_canvas.setPanZoom(data.panX, data.panY, data.zoom);

    m_statusLabel.set_text("Model loaded successfully");
    return "";
}

} // namespace Gui
