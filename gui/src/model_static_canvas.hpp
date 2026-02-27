#ifndef GUI_MODEL_STATIC_CANVAS_HPP
#define GUI_MODEL_STATIC_CANVAS_HPP

// General STD/STL headers
#include <optional>

// Constraint solver headers
#include <gcs_data_structures.hpp>

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief Read-only canvas showing a snapshot of the constraint model.
 *
 * Displays the constraint graph using canvas-space positions
 * (Point::canvasPosition, Line::canvasP1/canvasP2). The snapshot
 * is taken when @c setGraph() is called and does not update if
 * the original model changes afterwards.
 *
 * Supports pan (drag) and zoom (scroll).
 */
class ModelStaticCanvas : public Gtk::DrawingArea {
public:
    ModelStaticCanvas();

    /**
     * @brief Set the constraint graph to display.
     *
     * Makes a shallow copy of the graph (elements share the same
     * underlying data, but canvas positions are not modified by the
     * solver). Auto-fits the view to the model's bounding box.
     *
     * @param graph The constraint graph to snapshot.
     */
    void setGraph(const Gcs::ConstraintGraph& graph);

    /**
     * @brief Clear the displayed graph.
     */
    void clear();

private:
    void onDraw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);

    void setupControllers();
    void drawGrid(
        const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);

    std::optional<Gcs::ConstraintGraph> m_graph;

    // Pan / zoom state
    double m_panX = 0.0;
    double m_panY = 0.0;
    double m_zoom = 1.0;

    // Drag state
    double m_dragStartX = 0.0;
    double m_dragStartY = 0.0;
    double m_dragStartPanX = 0.0;
    double m_dragStartPanY = 0.0;

    // Gesture controllers (prevent premature destruction)
    Glib::RefPtr<Gtk::GestureDrag> m_dragController;
    Glib::RefPtr<Gtk::EventControllerScroll> m_scrollController;
};

} // namespace Gui

#endif // GUI_MODEL_STATIC_CANVAS_HPP
