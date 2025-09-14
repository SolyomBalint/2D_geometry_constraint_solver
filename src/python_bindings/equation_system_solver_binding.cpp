#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h> // Add this include
#include "../constraint_solver/constraint_equation_solver.hpp"

namespace nb = nanobind;

NB_MODULE(equation_system_solver_binding, m)
{
    m.doc() = "2D Geometry Constraint Solver Python Bindings";

    nb::class_<Solver::Coordinates2D>(m, "Coordinates2D")
        .def(nb::init<>())
        .def(nb::init<double, double>(), nb::arg("x"), nb::arg("y"))
        .def_rw("x", &Solver::Coordinates2D::x)
        .def_rw("y", &Solver::Coordinates2D::y)
        .def("__repr__", [](const Solver::Coordinates2D& c) -> std::string {
            return "Coordinates2D(x=" + std::to_string(c.x)
                + ", y=" + std::to_string(c.y) + ")";
        });

    m.def("calculatePointToPointDistanceTriangle",
        &Solver::calculatePointToPointDistanceTriangle,
        "Calculates the coordinates of edges of a triangle using "
        "Newton-Raphson method",
        nb::arg("xToYDistance"), nb::arg("xToZDistance"),
        nb::arg("yToZDistance"));
}
