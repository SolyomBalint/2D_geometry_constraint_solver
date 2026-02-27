#include "solver_properties_panel.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <ranges>
#include <vector>

// Constraint solver headers
#include <constraints.hpp>
#include <elements.hpp>
#include <solvers/heuristics.hpp>

namespace Gui {

namespace {
    constexpr int SIDEBAR_WIDTH = 220;
    constexpr int SECTION_MARGIN_TOP = 8;
    constexpr int ROW_MARGIN_START = 8;

    /**
     * @brief Sign function returning -1, 0, or 1.
     */
    int sign(double x)
    {
        return (x > 0) - (x < 0);
    }

    /**
     * @brief Format a sign as a string.
     */
    std::string signStr(double x)
    {
        int s = sign(x);
        if (s > 0)
            return "+";
        if (s < 0)
            return "-";
        return "0";
    }

    /**
     * @brief Format orientation as CW/CCW/collinear.
     */
    std::string orientationStr(double x)
    {
        if (x > 0)
            return "CCW";
        if (x < 0)
            return "CW";
        return "Collinear";
    }

    /**
     * @brief Collect elements by type from a component.
     *
     * Returns vectors of (NodeId, Element*) pairs for points
     * and lines, separated by solved/unsolved status.
     */
    struct ElementClassification {
        struct NodeElem {
            MathUtils::NodeId nodeId;
            Gcs::Element* elem;
        };
        std::vector<NodeElem> solvedPoints;
        std::vector<NodeElem> unsolvedPoints;
        std::vector<NodeElem> solvedLines;
        std::vector<NodeElem> unsolvedLines;
    };

    ElementClassification classifyElements(
        const Gcs::ConstraintGraph& component)
    {
        ElementClassification result;
        const auto& graph = component.getGraph();
        const auto& elementMap = component.getElementMap();

        for (const auto& node : graph.getNodes()) {
            auto elemResult = elementMap.get(node);
            if (!elemResult.has_value())
                continue;
            auto& elem = elemResult.value().get();
            auto* rawPtr = elem.get();

            if (rawPtr->isElementType<Gcs::Point>()) {
                if (rawPtr->isElementSet()) {
                    result.solvedPoints.push_back({ node, rawPtr });
                } else {
                    result.unsolvedPoints.push_back({ node, rawPtr });
                }
            } else if (rawPtr->isElementType<Gcs::Line>()) {
                if (rawPtr->isElementSet()) {
                    result.solvedLines.push_back({ node, rawPtr });
                } else {
                    result.unsolvedLines.push_back({ node, rawPtr });
                }
            }
        }
        return result;
    }
} // namespace

SolverPropertiesPanel::SolverPropertiesPanel()
    : m_contentBox(Gtk::Orientation::VERTICAL)
{
    set_policy(Gtk::PolicyType::NEVER, Gtk::PolicyType::AUTOMATIC);
    set_size_request(SIDEBAR_WIDTH, -1);
    set_hexpand(false);
    add_css_class("solver-sidebar");

    m_contentBox.set_margin(4);
    m_contentBox.set_spacing(2);

    set_child(m_contentBox);

    clear();
}

void SolverPropertiesPanel::clear()
{
    // Remove all children
    while (auto* child = m_contentBox.get_first_child()) {
        m_contentBox.remove(*child);
    }

    addSectionHeader("Solver Properties");
    addTextRow("No step selected.");
    addTextRow("Click \"Prepare\" then \"Step\" to begin.");
}

void SolverPropertiesPanel::updateForStep(const Gcs::ConstraintGraph& component,
    const Gcs::SolveResult& result, const std::string& solverName,
    const std::string& heuristicType, int stepIndex, int totalSteps)
{
    // Clear existing content
    while (auto* child = m_contentBox.get_first_child()) {
        m_contentBox.remove(*child);
    }

    // ---- Section 1: Component Info ----
    addSectionHeader("Component Info");
    addPropertyRow("Step", std::format("{} / {}", stepIndex + 1, totalSteps));
    addPropertyRow("Nodes", std::format("{}", component.nodeCount()));
    addPropertyRow("Edges", std::format("{}", component.edgeCount()));

    addPropertyRow("Solver", solverName);

    std::string statusStr;
    switch (result.status) {
    case Gcs::SolveStatus::Success:
        statusStr = "Success";
        break;
    case Gcs::SolveStatus::Unsupported:
        statusStr = "Unsupported";
        break;
    case Gcs::SolveStatus::Failed:
        statusStr = "Failed";
        break;
    }
    addPropertyRow("Status", statusStr);

    if (!result.message.empty()) {
        addPropertyRow("Message", result.message);
    }

    addPropertyRow(
        "Virtual edges", std::format("{}", component.getVirtualEdges().size()));
    addPropertyRow("Solved elements",
        std::format("{}", component.numberOfSolvedElements()));

    // ---- Section 2: Element Positions ----
    buildElementSection(component);

    // ---- Section 3: Heuristic Analysis ----
    if (result.status == Gcs::SolveStatus::Success) {
        buildHeuristicSection(component, heuristicType);
    }

    m_contentBox.show();
}

void SolverPropertiesPanel::buildElementSection(
    const Gcs::ConstraintGraph& component)
{
    addSectionHeader("Element Positions");

    const auto& graph = component.getGraph();
    const auto& elementMap = component.getElementMap();

    // Sort nodes by ID for deterministic display
    std::vector<MathUtils::NodeId> nodes;
    for (const auto& node : graph.getNodes()) {
        nodes.push_back(node);
    }
    std::ranges::sort(
        nodes, [](const auto& a, const auto& b) { return a.value < b.value; });

    for (const auto& node : nodes) {
        auto elemResult = elementMap.get(node);
        if (!elemResult.has_value())
            continue;

        const auto& elem = *elemResult.value().get();
        std::string status = elem.isElementSet() ? "Solved" : "Unsolved";

        if (elem.isElementType<Gcs::Point>()) {
            const auto& pt = elem.getElement<Gcs::Point>();
            addPropertyRow(std::format("P{} ({})", node.value, status), "");
            addPropertyRow("  canvas",
                std::format("({:.2f}, {:.2f})", pt.canvasPosition.x(),
                    pt.canvasPosition.y()));
            if (elem.isElementSet()) {
                addPropertyRow("  solver",
                    std::format(
                        "({:.2f}, {:.2f})", pt.position.x(), pt.position.y()));
            }
        } else if (elem.isElementType<Gcs::Line>()) {
            const auto& ln = elem.getElement<Gcs::Line>();
            addPropertyRow(std::format("L{} ({})", node.value, status), "");
            addPropertyRow("  canvas p1",
                std::format(
                    "({:.2f}, {:.2f})", ln.canvasP1.x(), ln.canvasP1.y()));
            addPropertyRow("  canvas p2",
                std::format(
                    "({:.2f}, {:.2f})", ln.canvasP2.x(), ln.canvasP2.y()));
            if (elem.isElementSet()) {
                addPropertyRow("  solver p1",
                    std::format("({:.2f}, {:.2f})", ln.p1.x(), ln.p1.y()));
                addPropertyRow("  solver p2",
                    std::format("({:.2f}, {:.2f})", ln.p2.x(), ln.p2.y()));
            }
        }
    }

    // Show constraints
    addSectionHeader("Constraints");
    for (const auto& edge : graph.getEdges()) {
        auto [nodeA, nodeB] = graph.getEndpoints(edge);
        auto constraint = component.getConstraintForEdge(edge);
        bool isVirtual = component.isVirtualEdge(edge);

        std::string label;
        if (isVirtual) {
            label = std::format("E{}: {} -- {} [Virtual]", edge.value,
                nodeA.value, nodeB.value);
        } else if (constraint) {
            auto val = constraint->getConstraintValue();
            std::string cName = constraint->getConstraintName();
            if (val.has_value()) {
                if (constraint->isConstraintType<Gcs::AngleConstraint>()) {
                    double deg = val.value() * 180.0 / std::numbers::pi;
                    label = std::format("E{}: {} -- {} [{}={:.1f}\u00B0]",
                        edge.value, nodeA.value, nodeB.value, cName, deg);
                } else {
                    label = std::format("E{}: {} -- {} [{}={:.2f}]", edge.value,
                        nodeA.value, nodeB.value, cName, val.value());
                }
            } else {
                label = std::format("E{}: {} -- {} [{}]", edge.value,
                    nodeA.value, nodeB.value, cName);
            }
        }
        addTextRow(label);
    }
}

void SolverPropertiesPanel::buildHeuristicSection(
    const Gcs::ConstraintGraph& component, const std::string& heuristicType)
{
    addSectionHeader("Heuristic Analysis");

    if (heuristicType == "triangleOrientation") {
        buildTriangleOrientationHeuristic(component);
    } else if (heuristicType == "lineSignedDistances") {
        buildLineSignedDistancesHeuristic(component);
    } else if (heuristicType == "lineNormalAngle") {
        buildLineNormalAngleHeuristic(component);
    } else {
        addTextRow("No heuristic data available.");
    }
}

void SolverPropertiesPanel::buildTriangleOrientationHeuristic(
    const Gcs::ConstraintGraph& component)
{
    addPropertyRow("Heuristic", "pickByTriangleOrientation");

    auto cls = classifyElements(component);

    // Determine A, B (fixed/anchor), Free based on solver type
    Eigen::Vector2d canvasA;
    Eigen::Vector2d canvasB;
    Eigen::Vector2d canvasFree;
    Eigen::Vector2d solverA;
    Eigen::Vector2d solverB;
    Eigen::Vector2d solverFree;

    bool found = false;

    if (cls.solvedPoints.size() == 3 && cls.unsolvedPoints.empty()) {
        // All 3 points are now solved — determine which was
        // the "free" one. For ZeroFixedPointsTriangleSolver,
        // the first two (sorted by ID) were anchored.
        // For TwoFixedPointsDistanceSolver, the 2
        // previously-set ones were fixed.
        auto allPoints = cls.solvedPoints;
        std::ranges::sort(allPoints, [](const auto& a, const auto& b) {
            return a.nodeId.value < b.nodeId.value;
        });

        // The solver anchors the first two points and solves
        // the third. For the partially-solved case, we can't
        // easily reconstruct which two were "previously
        // solved", but we can still compute the orientation
        // for all three.
        auto& ptA = allPoints[0].elem->getElement<Gcs::Point>();
        auto& ptB = allPoints[1].elem->getElement<Gcs::Point>();
        auto& ptFree = allPoints[2].elem->getElement<Gcs::Point>();

        canvasA = ptA.canvasPosition;
        canvasB = ptB.canvasPosition;
        canvasFree = ptFree.canvasPosition;
        solverA = ptA.position;
        solverB = ptB.position;
        solverFree = ptFree.position;
        found = true;

    } else if (cls.solvedPoints.size() >= 1 && cls.solvedLines.size() >= 1) {
        // FixedPointAndLineFreePointSolver or
        // TwoFixedLinesFreePointSolver

        if (cls.solvedPoints.size() >= 2 && cls.solvedLines.size() >= 1) {
            // Point + line + point scenario — find the "free"
            // point (the one that was last solved)
            auto& ptA = cls.solvedPoints[0].elem->getElement<Gcs::Point>();
            auto& ln = cls.solvedLines[0].elem->getElement<Gcs::Line>();
            auto& ptFree = cls.solvedPoints[1].elem->getElement<Gcs::Point>();

            canvasA = ptA.canvasPosition;
            canvasB = (ln.canvasP1 + ln.canvasP2) / 2.0;
            canvasFree = ptFree.canvasPosition;
            solverA = ptA.position;
            solverB = ln.midpoint();
            solverFree = ptFree.position;
            found = true;

        } else if (cls.solvedLines.size() >= 2
            && cls.solvedPoints.size() >= 1) {
            // Two lines + point scenario
            auto& ln1 = cls.solvedLines[0].elem->getElement<Gcs::Line>();
            auto& ln2 = cls.solvedLines[1].elem->getElement<Gcs::Line>();
            auto& ptFree = cls.solvedPoints[0].elem->getElement<Gcs::Point>();

            canvasA = (ln1.canvasP1 + ln1.canvasP2) / 2.0;
            canvasB = (ln2.canvasP1 + ln2.canvasP2) / 2.0;
            canvasFree = ptFree.canvasPosition;
            solverA = ln1.midpoint();
            solverB = ln2.midpoint();
            solverFree = ptFree.position;
            found = true;
        }
    }

    if (!found) {
        addTextRow("Could not extract triangle vertices.");
        return;
    }

    using Gcs::Solvers::triangleOrientation;

    double canvasOri = triangleOrientation(canvasA, canvasB, canvasFree);
    double solverOri = triangleOrientation(solverA, solverB, solverFree);

    addPropertyRow(
        "Canvas A", std::format("({:.2f}, {:.2f})", canvasA.x(), canvasA.y()));
    addPropertyRow(
        "Canvas B", std::format("({:.2f}, {:.2f})", canvasB.x(), canvasB.y()));
    addPropertyRow("Canvas Free",
        std::format("({:.2f}, {:.2f})", canvasFree.x(), canvasFree.y()));

    addPropertyRow("Canvas orientation",
        std::format("{:.4f} ({})", canvasOri, orientationStr(canvasOri)));

    addPropertyRow(
        "Solver A", std::format("({:.2f}, {:.2f})", solverA.x(), solverA.y()));
    addPropertyRow(
        "Solver B", std::format("({:.2f}, {:.2f})", solverB.x(), solverB.y()));
    addPropertyRow("Solver Free",
        std::format("({:.2f}, {:.2f})", solverFree.x(), solverFree.y()));

    addPropertyRow("Solver orientation",
        std::format("{:.4f} ({})", solverOri, orientationStr(solverOri)));

    bool signsMatch = (sign(canvasOri) == sign(solverOri));
    addPropertyRow("Decision",
        signsMatch ? "Signs match -> candidate accepted"
                   : "Signs differ -> mirror candidate used");
}

void SolverPropertiesPanel::buildLineSignedDistancesHeuristic(
    const Gcs::ConstraintGraph& component)
{
    addPropertyRow("Heuristic", "pickLineBySignedDistances");

    auto cls = classifyElements(component);

    // We need 2 points and 1 line (all solved at this point)
    std::vector<ElementClassification::NodeElem> points;
    std::vector<ElementClassification::NodeElem> lines;

    for (auto& sp : cls.solvedPoints)
        points.push_back(sp);
    for (auto& sl : cls.solvedLines)
        lines.push_back(sl);

    if (points.size() < 2 || lines.empty()) {
        addTextRow("Could not extract point-point-line config.");
        return;
    }

    const auto& pt1 = points[0].elem->getElement<Gcs::Point>();
    const auto& pt2 = points[1].elem->getElement<Gcs::Point>();
    const auto& line = lines[0].elem->getElement<Gcs::Line>();

    // Canvas signed distances
    using Gcs::Solvers::signedDistanceToLine;
    double canvasDist1 = signedDistanceToLine(
        pt1.canvasPosition, line.canvasP1, line.canvasP2);
    double canvasDist2 = signedDistanceToLine(
        pt2.canvasPosition, line.canvasP1, line.canvasP2);

    // Solver signed distances (using solved line endpoints)
    double solverDist1 = signedDistanceToLine(pt1.position, line.p1, line.p2);
    double solverDist2 = signedDistanceToLine(pt2.position, line.p1, line.p2);

    addPropertyRow("Point 1 canvas",
        std::format("({:.2f}, {:.2f})", pt1.canvasPosition.x(),
            pt1.canvasPosition.y()));
    addPropertyRow("Point 2 canvas",
        std::format("({:.2f}, {:.2f})", pt2.canvasPosition.x(),
            pt2.canvasPosition.y()));
    addPropertyRow("Line canvas",
        std::format("({:.2f},{:.2f})->({:.2f},{:.2f})", line.canvasP1.x(),
            line.canvasP1.y(), line.canvasP2.x(), line.canvasP2.y()));

    addPropertyRow("Canvas signedDist P1",
        std::format("{:.4f} ({})", canvasDist1, signStr(canvasDist1)));
    addPropertyRow("Canvas signedDist P2",
        std::format("{:.4f} ({})", canvasDist2, signStr(canvasDist2)));

    addPropertyRow("Point 1 solver",
        std::format("({:.2f}, {:.2f})", pt1.position.x(), pt1.position.y()));
    addPropertyRow("Point 2 solver",
        std::format("({:.2f}, {:.2f})", pt2.position.x(), pt2.position.y()));
    addPropertyRow("Line solver",
        std::format("({:.2f},{:.2f})->({:.2f},{:.2f})", line.p1.x(),
            line.p1.y(), line.p2.x(), line.p2.y()));

    addPropertyRow("Solver signedDist P1",
        std::format("{:.4f} ({})", solverDist1, signStr(solverDist1)));
    addPropertyRow("Solver signedDist P2",
        std::format("{:.4f} ({})", solverDist2, signStr(solverDist2)));

    bool sign1Match = (sign(canvasDist1) == sign(solverDist1));
    bool sign2Match = (sign(canvasDist2) == sign(solverDist2));

    addPropertyRow("Decision",
        std::format("P1 sign {}, P2 sign {}", sign1Match ? "match" : "differ",
            sign2Match ? "match" : "differ"));
}

void SolverPropertiesPanel::buildLineNormalAngleHeuristic(
    const Gcs::ConstraintGraph& component)
{
    addPropertyRow("Heuristic", "pickLineNormalByAngleOrientation");

    auto cls = classifyElements(component);

    // Need 2 lines (at least 1 solved — the other was just
    // solved in this step)
    std::vector<ElementClassification::NodeElem> allLines;
    for (auto& sl : cls.solvedLines)
        allLines.push_back(sl);

    if (allLines.size() < 2) {
        addTextRow("Could not extract line-line configuration.");
        return;
    }

    // Determine fixed vs free line. The "fixed" line is the one
    // that has its canvasP1/P2 positions close to its solver
    // positions (it was solved earlier and its positions are
    // established). We heuristically pick the first as fixed.
    const auto& line1 = allLines[0].elem->getElement<Gcs::Line>();
    const auto& line2 = allLines[1].elem->getElement<Gcs::Line>();

    Eigen::Vector2d canvasFixedDir = line1.canvasP2 - line1.canvasP1;
    Eigen::Vector2d canvasFreeDir = line2.canvasP2 - line2.canvasP1;

    // Check if angle constraint has flipOrientation
    const auto& graph = component.getGraph();
    bool flipped = false;
    for (const auto& edge : graph.getEdges()) {
        if (component.isVirtualEdge(edge))
            continue;
        auto constr = component.getConstraintForEdge(edge);
        if (constr && constr->isConstraintType<Gcs::AngleConstraint>()) {
            auto* ac = constr->getConstraintAs<Gcs::AngleConstraint>();
            if (ac != nullptr) {
                flipped = ac->flipOrientation;
            }
            break;
        }
    }

    if (flipped) {
        canvasFreeDir = -canvasFreeDir;
    }

    // Canvas cross product: fixedDir x freeDir
    double canvasCross = canvasFixedDir.x() * canvasFreeDir.y()
        - canvasFixedDir.y() * canvasFreeDir.x();

    // Solver free line direction from solved endpoints
    Eigen::Vector2d solverFreeDir = line2.p2 - line2.p1;

    // Cross product: canvasFixedDir x solverFreeDir
    double solverCross = canvasFixedDir.x() * solverFreeDir.y()
        - canvasFixedDir.y() * solverFreeDir.x();

    addPropertyRow("Fixed line canvas dir",
        std::format(
            "({:.2f}, {:.2f})", canvasFixedDir.x(), canvasFixedDir.y()));
    addPropertyRow("Free line canvas dir",
        std::format("({:.2f}, {:.2f})", canvasFreeDir.x(), canvasFreeDir.y()));
    addPropertyRow("Flip orientation", flipped ? "Yes" : "No");

    addPropertyRow("Canvas cross product",
        std::format("{:.4f} ({})", canvasCross, signStr(canvasCross)));

    addPropertyRow("Solver free line dir",
        std::format("({:.2f}, {:.2f})", solverFreeDir.x(), solverFreeDir.y()));
    addPropertyRow("Solver cross product",
        std::format("{:.4f} ({})", solverCross, signStr(solverCross)));

    bool signsMatch = (sign(canvasCross) == sign(solverCross));
    addPropertyRow("Decision",
        signsMatch ? "Signs match -> candidate accepted"
                   : "Signs differ -> mirror candidate used");
}

void SolverPropertiesPanel::addSectionHeader(const std::string& title)
{
    auto* label = Gtk::make_managed<Gtk::Label>();
    label->set_markup(std::format("<b>{}</b>", title));
    label->set_halign(Gtk::Align::START);
    label->set_margin_top(SECTION_MARGIN_TOP);
    label->set_margin_start(ROW_MARGIN_START);
    m_contentBox.append(*label);

    auto* sep = Gtk::make_managed<Gtk::Separator>(Gtk::Orientation::HORIZONTAL);
    sep->set_margin_start(ROW_MARGIN_START);
    sep->set_margin_end(4);
    m_contentBox.append(*sep);
}

void SolverPropertiesPanel::addPropertyRow(
    const std::string& key, const std::string& value)
{
    auto* row = Gtk::make_managed<Gtk::Box>(Gtk::Orientation::HORIZONTAL);
    row->set_spacing(4);
    row->set_margin_start(ROW_MARGIN_START);

    auto* keyLabel = Gtk::make_managed<Gtk::Label>(key);
    keyLabel->set_halign(Gtk::Align::START);
    keyLabel->add_css_class("dim-label");
    row->append(*keyLabel);

    if (!value.empty()) {
        auto* valLabel = Gtk::make_managed<Gtk::Label>(value);
        valLabel->set_halign(Gtk::Align::END);
        valLabel->set_hexpand(true);
        valLabel->set_wrap(true);
        valLabel->set_wrap_mode(Pango::WrapMode::WORD_CHAR);
        valLabel->set_selectable(true);
        row->append(*valLabel);
    }

    m_contentBox.append(*row);
}

void SolverPropertiesPanel::addTextRow(const std::string& text)
{
    auto* label = Gtk::make_managed<Gtk::Label>(text);
    label->set_halign(Gtk::Align::START);
    label->set_margin_start(ROW_MARGIN_START);
    label->set_wrap(true);
    label->set_wrap_mode(Pango::WrapMode::WORD_CHAR);
    m_contentBox.append(*label);
}

} // namespace Gui
