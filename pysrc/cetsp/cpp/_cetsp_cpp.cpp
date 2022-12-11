//
// Created by Dominik Krupke on 11.12.22.
//
#include "cetsp/common.h"
#include "cetsp/heuristics.h"
#include <fmt/core.h>
#include <pybind11/operators.h> // to define operator overloading
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // automatic conversion of vectors
namespace py = pybind11;
using namespace cetsp;

PYBIND11_MODULE(_cetsp_cpp, m) {
  py::class_<Point>(m, "Point", "Simple position")
      .def(py::init<double, double>())
      .def_readwrite("x", &Point::x)
      .def_readwrite("y", &Point::y)
      .def("dist", &Point::dist)
      .def("__repr__", [](const Point &point) {
        return fmt::format("Point({}, {})", point.x, point.y);
      });

  py::class_<Circle>(m, "Circle", "Circle with center and radius")
      .def(py::init<Point, double>())
      .def_readwrite("center", &Circle::center)
      .def_readwrite("radius", &Circle::radius)
      .def("contains", &Circle::contains)
      .def("__repr__", [](const Circle &self) {
        return fmt::format("Circle(({}, {}), {})", self.center.x, self.center.y,
                           self.radius);
      });

  py::class_<Trajectory>(m, "Trajectory", "Trajectory of a solution")
      .def(py::init<std::vector<Point>>())
      .def("length", &Trajectory::length)
      .def("is_tour", &Trajectory::is_tour)
      .def("__len__", [](const Trajectory &self) { return self.points.size(); })
      .def("__getitem__",
           [](const Trajectory &self, int i) { return self.points.at(i); });
  m.def("compute_tour_by_2opt", py::overload_cast<std::vector<Circle>, bool>(&compute_tour_by_2opt));
}
