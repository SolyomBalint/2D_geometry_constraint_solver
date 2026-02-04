#ifndef GUI_DECOMPOSITION_CANVAS_HPP
#define GUI_DECOMPOSITION_CANVAS_HPP

// General STD/STL headers
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
 * @brief Canvas that renders leaf components of an S-tree decomposition.
 *
 * Extracts leaf ConstraintGraphs from the S-tree and renders them on a
 * single free-form canvas. Each leaf component is drawn using its
 * elements' original canvas positions, with a distinct color per
 * component. Virtual edges are rendered as purple dashed lines.
 * Supports global pan and zoom.
 */
class DecompositionCanvas : public Gtk::DrawingArea {
public:
    DecompositionCanvas();

    /**
     * @brief Set the S-tree to visualize and extract its leaves.
     * @param stree The decomposition S-tree.
     */
    void setDecomposition(
        const MathUtils::BinaryTree<Gcs::ConstraintGraph>& stree);

    /**
     * @brief Clear the current decomposition data.
     */
    void clear();

private:
    void onDraw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);

    void drawGrid(
        const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);

    // Coordinate transforms
    void screenToWorld(double sx, double sy, double& wx, double& wy) const;

    // Event handlers
    void onDragBegin(double startX, double startY);
    void onDragUpdate(double offsetX, double offsetY);
    void onDragEnd(double offsetX, double offsetY);
    bool onScroll(double dx, double dy);
    void onMotion(double x, double y);

    std::vector<Gcs::ConstraintGraph> m_leafGraphs;

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

#endif // GUI_DECOMPOSITION_CANVAS_HPP
