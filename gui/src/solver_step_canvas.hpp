#ifndef GUI_SOLVER_STEP_CANVAS_HPP
#define GUI_SOLVER_STEP_CANVAS_HPP

// General STD/STL headers
#include <vector>

// Constraint solver headers
#include <gcs_data_structures.hpp>

// Thirdparty headers
#include <gtkmm.h>

namespace Gui {

/**
 * @brief Canvas showing step-by-step solver-space reconstruction.
 *
 * Starts as an empty coordinate system with a 0,0 origin marker.
 * After each solve step, the newly solved component is added and
 * rendered in a bright accent color, while previously solved
 * components are rendered dimmed (gray).
 *
 * Uses solver-space positions (Point::position, Line::p1/p2)
 * rather than canvas positions.
 *
 * Supports pan (drag) and zoom (scroll).
 */
class SolverStepCanvas : public Gtk::DrawingArea {
public:
    SolverStepCanvas();

    /**
     * @brief Add a solved component to be rendered.
     *
     * The component must already have been solved (elements with
     * isElementSet() == true). It is stored by pointer; the
     * caller must ensure the pointed-to graph outlives this canvas.
     *
     * @param component Pointer to the solved component graph.
     */
    void addSolvedComponent(const Gcs::ConstraintGraph* component);

    /**
     * @brief Clear all solved components and reset the view.
     */
    void clear();

    /**
     * @brief Auto-fit the view to encompass all solved elements.
     */
    void autoFit();

private:
    void onDraw(const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);

    void setupControllers();
    void drawGrid(
        const Cairo::RefPtr<Cairo::Context>& cr, int width, int height);
    void drawOriginMarker(const Cairo::RefPtr<Cairo::Context>& cr);

    std::vector<const Gcs::ConstraintGraph*> m_solvedComponents;

    // Pan / zoom state
    double m_panX = 0.0;
    double m_panY = 0.0;
    double m_zoom = 1.0;
    bool m_hasAutoFit = false;

    // Drag state
    double m_dragStartX = 0.0;
    double m_dragStartY = 0.0;
    double m_dragStartPanX = 0.0;
    double m_dragStartPanY = 0.0;

    // Gesture controllers
    Glib::RefPtr<Gtk::GestureDrag> m_dragController;
    Glib::RefPtr<Gtk::EventControllerScroll> m_scrollController;
};

} // namespace Gui

#endif // GUI_SOLVER_STEP_CANVAS_HPP
