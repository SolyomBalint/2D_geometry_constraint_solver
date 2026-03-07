#include "dr_plan_canvas.hpp"

// General STD/STL headers
#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <string>
#include <type_traits>
#include <variant>

namespace Gui {

namespace {

    constexpr double ZOOM_FACTOR = 1.1;
    constexpr double MIN_ZOOM = 0.05;
    constexpr double MAX_ZOOM = 20.0;

    constexpr double NODE_WIDTH = 250.0;
    constexpr double NODE_HEIGHT = 200.0;
    constexpr double HORIZONTAL_GAP = 26.0;
    constexpr double VERTICAL_GAP = 34.0;
    constexpr double TREE_GAP = 90.0;

    constexpr double HEADER_FONT_SIZE = 11.0;
    constexpr double DETAIL_FONT_SIZE = 10.0;
    constexpr double META_FONT_SIZE = 9.0;
    constexpr double EDGE_WIDTH = 1.6;
    constexpr double GRAPH_PADDING = 10.0;
    constexpr double HEADER_SECTION_HEIGHT = 42.0;

} // namespace

std::size_t DRPlanCanvas::PlanNodeRefHash::operator()(
    const PlanNodeRef& ref) const noexcept
{
    const std::size_t treeHash = std::hash<std::size_t> {}(ref.treeIndex);
    const std::size_t nodeHash
        = std::hash<MathUtils::GeneralTreeNodeId> {}(ref.nodeId);
    return treeHash
        ^ (nodeHash + 0x9e3779b9 + (treeHash << 6) + (treeHash >> 2));
}

DRPlanCanvas::DRPlanCanvas()
{
    set_hexpand(true);
    set_vexpand(true);

    set_draw_func(sigc::mem_fun(*this, &DRPlanCanvas::onDraw));

    m_dragGesture = Gtk::GestureDrag::create();
    m_dragGesture->set_button(GDK_BUTTON_PRIMARY);
    m_dragGesture->signal_drag_begin().connect(
        sigc::mem_fun(*this, &DRPlanCanvas::onDragBegin));
    m_dragGesture->signal_drag_update().connect(
        sigc::mem_fun(*this, &DRPlanCanvas::onDragUpdate));
    m_dragGesture->signal_drag_end().connect(
        sigc::mem_fun(*this, &DRPlanCanvas::onDragEnd));
    add_controller(m_dragGesture);

    m_scrollController = Gtk::EventControllerScroll::create();
    m_scrollController->set_flags(Gtk::EventControllerScroll::Flags::VERTICAL);
    m_scrollController->signal_scroll().connect(
        sigc::mem_fun(*this, &DRPlanCanvas::onScroll), false);
    add_controller(m_scrollController);

    m_motionController = Gtk::EventControllerMotion::create();
    m_motionController->signal_motion().connect(
        sigc::mem_fun(*this, &DRPlanCanvas::onMotion));
    add_controller(m_motionController);
}

void DRPlanCanvas::setPlans(const std::vector<Gcs::PlanTree>& planRoots,
    const Gcs::ConstraintGraph& sourceGraph)
{
    m_planRoots = planRoots;
    m_sourceGraph = sourceGraph;
    m_originalIds = buildOriginalIdMap(sourceGraph);
    computeLayout();

    if (m_layout.empty()) {
        queue_draw();
        return;
    }

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();

    for (const auto& [nodeRef, layout] : m_layout) {
        (void)nodeRef;
        minX = std::min(minX, layout.x - layout.width / 2.0);
        minY = std::min(minY, layout.y - layout.height / 2.0);
        maxX = std::max(maxX, layout.x + layout.width / 2.0);
        maxY = std::max(maxY, layout.y + layout.height / 2.0);
    }

    const double centerX = (minX + maxX) / 2.0;
    const double centerY = (minY + maxY) / 2.0;

    int width = get_width();
    int height = get_height();
    if (width <= 0) {
        width = 800;
    }
    if (height <= 0) {
        height = 600;
    }

    const double rangeX = maxX - minX;
    const double rangeY = maxY - minY;
    if (rangeX > 0.0 && rangeY > 0.0) {
        const double zoomX = static_cast<double>(width) / rangeX * 0.88;
        const double zoomY = static_cast<double>(height) / rangeY * 0.88;
        m_zoom = std::clamp(std::min(zoomX, zoomY), MIN_ZOOM, MAX_ZOOM);
    } else {
        m_zoom = 1.0;
    }

    m_panX = static_cast<double>(width) / 2.0 - centerX * m_zoom;
    m_panY = static_cast<double>(height) / 2.0 - centerY * m_zoom;

    queue_draw();
}

void DRPlanCanvas::setEmptyMessage(std::string message)
{
    m_emptyMessage = std::move(message);
    queue_draw();
}

void DRPlanCanvas::clear()
{
    m_planRoots.clear();
    m_sourceGraph.reset();
    m_originalIds.clear();
    m_layout.clear();
    m_edges.clear();
    m_panX = 0.0;
    m_panY = 0.0;
    m_zoom = 1.0;
    queue_draw();
}

void DRPlanCanvas::computeLayout()
{
    m_layout.clear();
    m_edges.clear();

    double currentLeft = 0.0;
    for (std::size_t treeIndex = 0; treeIndex < m_planRoots.size();
        ++treeIndex) {
        const auto& tree = m_planRoots[treeIndex];
        const auto root = tree.getRoot();
        if (!root.has_value()) {
            continue;
        }

        const double rightBound
            = layoutSubtree(tree, root.value(), treeIndex, currentLeft, 0.0);
        currentLeft = rightBound + TREE_GAP;
    }
}

double DRPlanCanvas::layoutSubtree(const Gcs::PlanTree& tree,
    MathUtils::GeneralTreeNodeId nodeId, std::size_t treeIndex,
    double leftBound, double depth)
{
    const auto children = tree.children(nodeId);
    const double y = depth * (NODE_HEIGHT + VERTICAL_GAP);

    if (children.empty()) {
        const double x = leftBound + NODE_WIDTH / 2.0;
        m_layout.emplace(
            PlanNodeRef {
                .treeIndex = treeIndex,
                .nodeId = nodeId,
            },
            NodeLayout {
                .x = x,
                .y = y,
                .width = NODE_WIDTH,
                .height = NODE_HEIGHT,
            });
        return leftBound + NODE_WIDTH + HORIZONTAL_GAP;
    }

    double childLeft = leftBound;
    std::vector<double> childCenters;
    childCenters.reserve(children.size());

    for (const auto& childId : children) {
        childLeft
            = layoutSubtree(tree, childId, treeIndex, childLeft, depth + 1.0);

        const PlanNodeRef childRef {
            .treeIndex = treeIndex,
            .nodeId = childId,
        };
        if (m_layout.contains(childRef)) {
            childCenters.push_back(m_layout.at(childRef).x);
            m_edges.emplace_back(
                PlanNodeRef {
                    .treeIndex = treeIndex,
                    .nodeId = nodeId,
                },
                childRef);
        }
    }

    double x = leftBound + NODE_WIDTH / 2.0;
    if (!childCenters.empty()) {
        const auto [minChild, maxChild]
            = std::minmax_element(childCenters.begin(), childCenters.end());
        x = (*minChild + *maxChild) / 2.0;
        x = std::max(x, leftBound + NODE_WIDTH / 2.0);
    }

    m_layout.emplace(
        PlanNodeRef {
            .treeIndex = treeIndex,
            .nodeId = nodeId,
        },
        NodeLayout {
            .x = x,
            .y = y,
            .width = NODE_WIDTH,
            .height = NODE_HEIGHT,
        });

    return std::max(childLeft, x + NODE_WIDTH / 2.0 + HORIZONTAL_GAP);
}

void DRPlanCanvas::onDraw(const Cairo::RefPtr<Cairo::Context>& cr,
    [[maybe_unused]] int width, [[maybe_unused]] int height)
{
    cr->set_source_rgb(0.96, 0.96, 0.96);
    cr->paint();

    if (m_layout.empty()) {
        cr->set_source_rgb(0.25, 0.25, 0.27);
        cr->set_font_size(14.0);

        Cairo::TextExtents extents;
        cr->get_text_extents(m_emptyMessage, extents);
        const double x = (static_cast<double>(width) - extents.width) / 2.0;
        const double y = (static_cast<double>(height) + extents.height) / 2.0;
        cr->move_to(x, y);
        cr->show_text(m_emptyMessage);
        return;
    }

    cr->save();
    cr->translate(m_panX, m_panY);
    cr->scale(m_zoom, m_zoom);

    drawEdges(cr);
    drawNodes(cr);

    cr->restore();
}

void DRPlanCanvas::drawEdges(const Cairo::RefPtr<Cairo::Context>& cr)
{
    cr->set_source_rgba(0.36, 0.38, 0.42, 0.9);
    cr->set_line_width(EDGE_WIDTH / m_zoom);

    for (const auto& [parentRef, childRef] : m_edges) {
        if (!m_layout.contains(parentRef) || !m_layout.contains(childRef)) {
            continue;
        }

        const auto& parent = m_layout.at(parentRef);
        const auto& child = m_layout.at(childRef);

        const double startX = parent.x;
        const double startY = parent.y + parent.height / 2.0;
        const double endX = child.x;
        const double endY = child.y - child.height / 2.0;

        cr->move_to(startX, startY);
        cr->line_to(endX, endY);
        cr->stroke();
    }
}

void DRPlanCanvas::drawNodes(const Cairo::RefPtr<Cairo::Context>& cr)
{
    for (std::size_t treeIndex = 0; treeIndex < m_planRoots.size();
        ++treeIndex) {
        const auto& tree = m_planRoots[treeIndex];
        const auto traversal = tree.traversePreOrder();

        for (const auto& nodeId : traversal) {
            const PlanNodeRef ref {
                .treeIndex = treeIndex,
                .nodeId = nodeId,
            };
            if (!m_layout.contains(ref)) {
                continue;
            }

            const auto& layout = m_layout.at(ref);
            const auto& planNode = tree.getValue(nodeId);

            const double left = layout.x - layout.width / 2.0;
            const double top = layout.y - layout.height / 2.0;

            double borderR = 0.35;
            double borderG = 0.35;
            double borderB = 0.35;
            double fillR = 1.0;
            double fillG = 1.0;
            double fillB = 1.0;

            if (planNode.kind == Gcs::PlanNodeKind::TrianglePrimitive) {
                borderR = 0.15;
                borderG = 0.55;
                borderB = 0.30;
                fillR = 0.92;
                fillG = 0.98;
                fillB = 0.93;
            } else if (planNode.kind == Gcs::PlanNodeKind::EdgePrimitive) {
                borderR = 0.55;
                borderG = 0.40;
                borderB = 0.12;
                fillR = 0.99;
                fillG = 0.95;
                fillB = 0.88;
            } else if (planNode.kind == Gcs::PlanNodeKind::Merge3) {
                borderR = 0.16;
                borderG = 0.35;
                borderB = 0.70;
                fillR = 0.92;
                fillG = 0.95;
                fillB = 1.0;
            } else {
                borderR = 0.60;
                borderG = 0.45;
                borderB = 0.15;
                fillR = 0.99;
                fillG = 0.96;
                fillB = 0.90;
            }

            cr->set_source_rgb(fillR, fillG, fillB);
            cr->rectangle(left, top, layout.width, layout.height);
            cr->fill();

            cr->set_source_rgb(borderR, borderG, borderB);
            cr->set_line_width(2.1 / m_zoom);
            cr->rectangle(left, top, layout.width, layout.height);
            cr->stroke();

            const std::string kindLabel = nodeKindLabel(planNode);
            const std::string detailLabel = nodeDetailLabel(planNode);
            const std::size_t childCount = tree.children(nodeId).size();
            const std::string treeIndexLabel = std::format(
                "root {} | children:{}", treeIndex + 1, childCount);

            cr->set_source_rgb(0.07, 0.07, 0.09);
            cr->set_font_size(HEADER_FONT_SIZE / m_zoom);

            Cairo::TextExtents kindExtents;
            cr->get_text_extents(kindLabel, kindExtents);
            cr->move_to(
                layout.x - kindExtents.width / 2.0, top + 18.0 / m_zoom);
            cr->show_text(kindLabel);

            cr->set_source_rgb(0.20, 0.22, 0.25);
            cr->set_font_size(DETAIL_FONT_SIZE / m_zoom);

            Cairo::TextExtents detailExtents;
            cr->get_text_extents(detailLabel, detailExtents);
            cr->move_to(
                layout.x - detailExtents.width / 2.0, top + 34.0 / m_zoom);
            cr->show_text(detailLabel);

            cr->set_source_rgba(0.78, 0.78, 0.78, 1.0);
            cr->set_line_width(0.8 / m_zoom);
            const double separatorY = top + HEADER_SECTION_HEIGHT / m_zoom;
            cr->move_to(left + GRAPH_PADDING / m_zoom, separatorY);
            cr->line_to(
                left + layout.width - GRAPH_PADDING / m_zoom, separatorY);
            cr->stroke();

            if (isPrimitiveNode(planNode)) {
                const auto primitiveGraph = buildPrimitiveSubgraph(planNode);
                if (primitiveGraph.has_value()
                    && primitiveGraph->nodeCount() > 0) {
                    const double graphAreaLeft = left + GRAPH_PADDING / m_zoom;
                    const double graphAreaTop = separatorY + 4.0 / m_zoom;
                    const double graphAreaWidth
                        = layout.width - 2.0 * GRAPH_PADDING / m_zoom;
                    const double graphAreaHeight = layout.height
                        - HEADER_SECTION_HEIGHT / m_zoom
                        - 2.0 * GRAPH_PADDING / m_zoom;

                    const auto bbox
                        = computeAbstractBoundingBox(primitiveGraph.value());
                    if (graphAreaWidth > 0.0 && graphAreaHeight > 0.0
                        && bbox.width() > 0.0 && bbox.height() > 0.0) {
                        const double scaleX = graphAreaWidth / bbox.width();
                        const double scaleY = graphAreaHeight / bbox.height();
                        const double scale = std::min(scaleX, scaleY) * 0.85;

                        const double areaCenterX
                            = graphAreaLeft + graphAreaWidth / 2.0;
                        const double areaCenterY
                            = graphAreaTop + graphAreaHeight / 2.0;

                        cr->save();
                        cr->rectangle(graphAreaLeft, graphAreaTop,
                            graphAreaWidth, graphAreaHeight);
                        cr->clip();

                        cr->translate(areaCenterX, areaCenterY);
                        cr->scale(scale, scale);

                        const double effectiveZoom = m_zoom * scale;
                        renderConstraintGraphAsGraph(cr, primitiveGraph.value(),
                            effectiveZoom, borderR, borderG, borderB,
                            &m_originalIds);

                        cr->restore();
                    }
                }
            }

            cr->set_source_rgb(0.25, 0.25, 0.25);
            cr->set_font_size(META_FONT_SIZE / m_zoom);
            cr->move_to(
                left + 8.0 / m_zoom, top + layout.height - 8.0 / m_zoom);
            cr->show_text(treeIndexLabel);
        }
    }
}

void DRPlanCanvas::onDragBegin(
    [[maybe_unused]] double startX, [[maybe_unused]] double startY)
{
    m_dragOrigPanX = m_panX;
    m_dragOrigPanY = m_panY;
}

void DRPlanCanvas::onDragUpdate(double offsetX, double offsetY)
{
    m_panX = m_dragOrigPanX + offsetX;
    m_panY = m_dragOrigPanY + offsetY;
    queue_draw();
}

void DRPlanCanvas::onDragEnd(
    [[maybe_unused]] double offsetX, [[maybe_unused]] double offsetY)
{
}

bool DRPlanCanvas::onScroll([[maybe_unused]] double dx, double dy)
{
    const double factor = (dy < 0.0) ? ZOOM_FACTOR : (1.0 / ZOOM_FACTOR);
    const double newZoom = std::clamp(m_zoom * factor, MIN_ZOOM, MAX_ZOOM);

    const double mouseWorldX = (m_mouseScreenX - m_panX) / m_zoom;
    const double mouseWorldY = (m_mouseScreenY - m_panY) / m_zoom;

    m_zoom = newZoom;
    m_panX = m_mouseScreenX - mouseWorldX * m_zoom;
    m_panY = m_mouseScreenY - mouseWorldY * m_zoom;
    queue_draw();
    return true;
}

void DRPlanCanvas::onMotion(double x, double y)
{
    m_mouseScreenX = x;
    m_mouseScreenY = y;
}

bool DRPlanCanvas::isPrimitiveNode(const Gcs::PlanNode& node)
{
    return node.kind == Gcs::PlanNodeKind::TrianglePrimitive
        || node.kind == Gcs::PlanNodeKind::EdgePrimitive;
}

std::vector<Gcs::ConstraintGraph::NodeIdType>
DRPlanCanvas::primitiveNodeElements(const Gcs::PlanNode& node) const
{
    return std::visit(
        [](const auto& info) -> std::vector<Gcs::ConstraintGraph::NodeIdType> {
            using InfoType = std::decay_t<decltype(info)>;

            if constexpr (std::is_same_v<InfoType, Gcs::EdgePrimitiveInfo>) {
                return { info.elements[0], info.elements[1] };
            }

            if constexpr (std::is_same_v<InfoType,
                              Gcs::TrianglePrimitiveInfo>) {
                return { info.elements[0], info.elements[1], info.elements[2] };
            }

            return {};
        },
        node.info);
}

std::optional<Gcs::ConstraintGraph> DRPlanCanvas::buildPrimitiveSubgraph(
    const Gcs::PlanNode& node) const
{
    if (!isPrimitiveNode(node) || !m_sourceGraph.has_value()) {
        return std::nullopt;
    }

    const auto elements = primitiveNodeElements(node);
    if (elements.size() < 2) {
        return std::nullopt;
    }

    Gcs::ConstraintGraph subgraph;
    std::unordered_map<Gcs::ConstraintGraph::NodeIdType,
        Gcs::ConstraintGraph::NodeIdType>
        sourceToLocal;

    for (const auto& elementNodeId : elements) {
        if (!m_sourceGraph->getGraph().hasNode(elementNodeId)) {
            continue;
        }

        const auto localNodeId = subgraph.getGraph().addNode();
        sourceToLocal.emplace(elementNodeId, localNodeId);

        const auto element = m_sourceGraph->getElement(elementNodeId);
        if (element) {
            (void)subgraph.addElement(localNodeId, element);
        }
    }

    for (std::size_t i = 0; i < elements.size(); ++i) {
        for (std::size_t j = i + 1; j < elements.size(); ++j) {
            const auto sourceEdge = m_sourceGraph->getGraph().getEdgeBetween(
                elements[i], elements[j]);
            if (!sourceEdge.has_value()) {
                continue;
            }

            if (!sourceToLocal.contains(elements[i])
                || !sourceToLocal.contains(elements[j])) {
                continue;
            }

            const auto localSourceI = sourceToLocal.at(elements[i]);
            const auto localSourceJ = sourceToLocal.at(elements[j]);

            if (m_sourceGraph->isVirtualEdge(sourceEdge.value())) {
                (void)subgraph.addVirtualEdge(localSourceI, localSourceJ);
                continue;
            }

            const auto localEdge
                = subgraph.getGraph().addEdge(localSourceI, localSourceJ);
            if (!localEdge.has_value()) {
                continue;
            }

            const auto constraint
                = m_sourceGraph->getConstraintForEdge(sourceEdge.value());
            if (constraint) {
                (void)subgraph.addConstraint(localEdge.value(), constraint);
            }
        }
    }

    return subgraph;
}

std::string DRPlanCanvas::nodeKindLabel(const Gcs::PlanNode& node)
{
    switch (node.kind) {
    case Gcs::PlanNodeKind::TrianglePrimitive:
        return "TrianglePrimitive";
    case Gcs::PlanNodeKind::EdgePrimitive:
        return "EdgePrimitive";
    case Gcs::PlanNodeKind::Merge3:
        return "Merge3";
    case Gcs::PlanNodeKind::Merge2:
        return "Merge2";
    }

    return "Unknown";
}

std::string DRPlanCanvas::nodeDetailLabel(const Gcs::PlanNode& node)
{
    return std::visit(
        [](const auto& info) -> std::string {
            using InfoType = std::decay_t<decltype(info)>;

            if constexpr (std::is_same_v<InfoType, Gcs::EdgePrimitiveInfo>) {
                return std::format("C{}: ({},{})", info.cluster.value,
                    info.elements[0].value, info.elements[1].value);
            }

            if constexpr (std::is_same_v<InfoType,
                              Gcs::TrianglePrimitiveInfo>) {
                return std::format("C{}: ({},{},{})", info.cluster.value,
                    info.elements[0].value, info.elements[1].value,
                    info.elements[2].value);
            }

            if constexpr (std::is_same_v<InfoType, Gcs::Merge3Info>) {
                return std::format("C{} <- C{}, C{}, C{}", info.output.value,
                    info.inputs[0].value, info.inputs[1].value,
                    info.inputs[2].value);
            }

            if constexpr (std::is_same_v<InfoType, Gcs::Merge2Info>) {
                return std::format("C{} <- C{}, C{}", info.output.value,
                    info.inputs[0].value, info.inputs[1].value);
            }

            return "Unknown";
        },
        node.info);
}

} // namespace Gui
