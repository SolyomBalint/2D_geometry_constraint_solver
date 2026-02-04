#ifndef GUI_STREE_CANVAS_HPP
#define GUI_STREE_CANVAS_HPP

// General STD/STL headers
#include <optional>
#include <unordered_map>
#include <vector>

// Custom headers
#include "./graph_renderer.hpp"

// Constraint solver headers
#include <gcs_data_structures.hpp>
#include <structures/binary_tree.hpp>

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief Canvas that renders the full S-tree decomposition as a tree
 *        diagram with miniature graph renderings at each node.
 *
 * Each tree node is drawn as a bordered rectangle containing a
 * scaled-to-fit rendering of its ConstraintGraph. Tree edges (lines)
 * connect parent nodes to their children. Leaf nodes have a distinct
 * border color. Supports global pan and zoom.
 */
class STreeCanvas : public Gtk::DrawingArea {
public:
    STreeCanvas();

    /**
     * @brief Set the S-tree to visualize.
     * @param stree The decomposition S-tree.
     */
    void setDecomposition(
        const MathUtils::BinaryTree<Gcs::ConstraintGraph>& stree);

    /**
     * @brief Clear the current decomposition data.
     */
    void clear();

private:
    /**
     * @brief Layout position for a tree node rectangle.
     */
    struct NodeLayout {
        double x = 0.0; ///< Center x of the rectangle in world coords.
        double y = 0.0; ///< Center y of the rectangle in world coords.
        double width = 0.0; ///< Width of the rectangle.
        double height = 0.0; ///< Height of the rectangle.
        bool isLeaf = false;
        int colorIndex = 0; ///< For leaf coloring.
    };

    void computeLayout();
    double layoutSubtree(MathUtils::TreeNodeId nodeId, double leftBound,
        double depth, int& leafCounter);

    void onDraw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);
    void drawTreeEdges(const Cairo::RefPtr<Cairo::Context>& cr);
    void drawTreeNodes(const Cairo::RefPtr<Cairo::Context>& cr);

    // Coordinate transforms
    void screenToWorld(double sx, double sy, double& wx, double& wy) const;

    // Event handlers
    void onDragBegin(double startX, double startY);
    void onDragUpdate(double offsetX, double offsetY);
    void onDragEnd(double offsetX, double offsetY);
    bool onScroll(double dx, double dy);
    void onMotion(double x, double y);

    MathUtils::BinaryTree<Gcs::ConstraintGraph> m_stree;
    std::unordered_map<MathUtils::TreeNodeId, NodeLayout> m_layout;

    // Pan and zoom
    double m_panX = 0.0;
    double m_panY = 0.0;
    double m_zoom = 1.0;
    double m_mouseScreenX = 0.0;
    double m_mouseScreenY = 0.0;

    // Dragging state (panning)
    double m_dragStartX = 0.0;
    double m_dragStartY = 0.0;
    double m_dragOrigPanX = 0.0;
    double m_dragOrigPanY = 0.0;

    // Gesture controllers
    Glib::RefPtr<Gtk::GestureDrag> m_dragGesture;
    Glib::RefPtr<Gtk::EventControllerScroll> m_scrollController;
    Glib::RefPtr<Gtk::EventControllerMotion> m_motionController;
};

} // namespace Gui

#endif // GUI_STREE_CANVAS_HPP
