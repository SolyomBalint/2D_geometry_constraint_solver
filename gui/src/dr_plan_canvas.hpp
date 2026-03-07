#ifndef GUI_DR_PLAN_CANVAS_HPP
#define GUI_DR_PLAN_CANVAS_HPP

// General STD/STL headers
#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Custom headers
#include "./graph_renderer.hpp"

// Constraint solver headers
#include <decomposition/bottom_up/producer_map.hpp>
#include <model/gcs_data_structures.hpp>

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief Canvas that renders one or more bottom-up DR-plan trees.
 */
class DRPlanCanvas : public Gtk::DrawingArea {
public:
    DRPlanCanvas();

    /**
     * @brief Set DR-plan roots to visualize.
     */
    void setPlans(const std::vector<Gcs::PlanTree>& planRoots,
        const Gcs::ConstraintGraph& sourceGraph);

    /**
     * @brief Set message shown when there is nothing to render.
     */
    void setEmptyMessage(std::string message);

    /**
     * @brief Clear current visualization data.
     */
    void clear();

private:
    struct PlanNodeRef {
        std::size_t treeIndex { 0 };
        MathUtils::GeneralTreeNodeId nodeId {};

        bool operator==(const PlanNodeRef&) const = default;
    };

    struct PlanNodeRefHash {
        std::size_t operator()(const PlanNodeRef& ref) const noexcept;
    };

    struct NodeLayout {
        double x = 0.0;
        double y = 0.0;
        double width = 220.0;
        double height = 74.0;
    };

    void computeLayout();
    double layoutSubtree(const Gcs::PlanTree& tree,
        MathUtils::GeneralTreeNodeId nodeId, std::size_t treeIndex,
        double leftBound, double depth);

    void onDraw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);
    void drawEdges(const Cairo::RefPtr<Cairo::Context>& cr);
    void drawNodes(const Cairo::RefPtr<Cairo::Context>& cr);

    void onDragBegin(double startX, double startY);
    void onDragUpdate(double offsetX, double offsetY);
    void onDragEnd(double offsetX, double offsetY);
    bool onScroll(double dx, double dy);
    void onMotion(double x, double y);

    [[nodiscard]] static std::string nodeKindLabel(const Gcs::PlanNode& node);
    [[nodiscard]] static std::string nodeDetailLabel(const Gcs::PlanNode& node);
    [[nodiscard]] static bool isPrimitiveNode(const Gcs::PlanNode& node);
    [[nodiscard]] std::vector<Gcs::ConstraintGraph::NodeIdType>
    primitiveNodeElements(const Gcs::PlanNode& node) const;
    [[nodiscard]] std::optional<Gcs::ConstraintGraph> buildPrimitiveSubgraph(
        const Gcs::PlanNode& node) const;

    std::vector<Gcs::PlanTree> m_planRoots;
    std::optional<Gcs::ConstraintGraph> m_sourceGraph;
    OriginalIdMap m_originalIds;
    std::unordered_map<PlanNodeRef, NodeLayout, PlanNodeRefHash> m_layout;
    std::vector<std::pair<PlanNodeRef, PlanNodeRef>> m_edges;
    std::string m_emptyMessage { "No DR-plan to display. Press Decompose." };

    double m_panX = 0.0;
    double m_panY = 0.0;
    double m_zoom = 1.0;
    double m_mouseScreenX = 0.0;
    double m_mouseScreenY = 0.0;

    double m_dragOrigPanX = 0.0;
    double m_dragOrigPanY = 0.0;

    Glib::RefPtr<Gtk::GestureDrag> m_dragGesture;
    Glib::RefPtr<Gtk::EventControllerScroll> m_scrollController;
    Glib::RefPtr<Gtk::EventControllerMotion> m_motionController;
};

} // namespace Gui

#endif // GUI_DR_PLAN_CANVAS_HPP
