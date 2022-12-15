//
// Created by Dominik Krupke on 15.12.22.
//
#include "cetsp/bnb.h"
#include "cetsp/common.h"
#include "cetsp/heuristics.h"
#include "cetsp/node.h"
#include <fmt/core.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h> // to define operator overloading
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // automatic conversion of vectors
namespace py = pybind11;
using namespace cetsp;

class PyNodeProcessor {
public:
  PyNodeProcessor(std::function<void(Node &)> &f) : f{&f} {}
  void process(Node &node) {
    if (f != nullptr) {
      (*f)(node);
    }
  }
  std::function<void(Node &)> *f;
};

Trajectory branch_and_bound(Instance &instance, std::function<void(Node &)> &f) {
  PyNodeProcessor pnp(f);
  Instance instance_ = instance;
  BranchAndBoundAlgorithm baba(&instance_, pnp); //, pnp);
  baba.optimize(30);
  if(!baba.get_solution()) {
    throw std::exception();
  }
  return *baba.get_solution();
}

PYBIND11_MODULE(_cetsp_bindings, m) {
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
  m.def("compute_tour_by_2opt",
        py::overload_cast<std::vector<Circle>, bool>(&compute_tour_by_2opt));
  m.def("compute_tour_from_sequence", py::overload_cast<const std::vector<Circle>&, bool>(&compute_tour));

  py::class_<Node>(m, "Node", "Node in the BnB-tree.")
      .def("get_lower_bound", &Node::get_lower_bound)
      .def("get_relaxed_solution", &Node::get_relaxed_solution)
      .def("add_lower_bound", &Node::add_lower_bound)
      .def("prune", &Node::prune)
      .def("is_pruned", &Node::is_pruned)
      .def("is_feasible", &Node::is_feasible)
      .def("get_fixed_sequence", &Node::get_fixed_sequence);

  py::class_<Instance>(m, "Instance", "CE-TSP Instance")
      .def(py::init<std::vector<Circle>>())
      .def("__init__",
           [](std::vector<Circle> &circles, Point &a, Point &b) -> Instance {
             Instance instance{circles};
             instance.path = {a, b};
             return instance;
           })
      .def("__len__", &Instance::size)
      .def("__getitem__",
           [](const Instance &self, int i) { return self.at(i); })
      .def("circles", [](const Instance &self) {
        std::vector<Circle> circles;
        for (const auto &c : self) {
          circles.push_back(c);
        }
        return circles;
      });
  m.def("branch_and_bound", &branch_and_bound);
}