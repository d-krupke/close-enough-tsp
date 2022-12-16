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

class PythonUserCallbacks : public DefaultUserCallbacks {
public:
  PythonUserCallbacks(std::function<void(EventContext)> *f) : f{f} {}

  void on_entering_node(EventContext& e) {
    EventContext e_ = e;
    if (f != nullptr) {
      (*f)(e_);
    }
  }

private:
  std::function<void(EventContext)> *f;
};

std::unique_ptr<Trajectory> branch_and_bound(Instance instance,
                            std::function<void(EventContext)> *f,
                            Trajectory &initial_solution, int timelimit) {
  PythonUserCallbacks pnp(f);
  BranchAndBoundAlgorithm baba(&instance, pnp);
  baba.add_upper_bound(initial_solution);
  baba.optimize(timelimit);
  return baba.get_solution();
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
           [](const Trajectory &self, int i) { return self.points.at(i); })
      .def("distance", &Trajectory::distance);
  m.def("compute_tour_by_2opt", &compute_tour_by_2opt);
  m.def("compute_tour_from_sequence",
        py::overload_cast<const std::vector<Circle> &, bool>(&compute_tour));

  py::class_<Node>(m, "Node", "Node in the BnB-tree.")
      .def("get_lower_bound", &Node::get_lower_bound)
      .def("get_relaxed_solution", &Node::get_relaxed_solution)
      .def("add_lower_bound", &Node::add_lower_bound)
      .def("prune", &Node::prune)
      .def("is_pruned", &Node::is_pruned)
      .def("is_feasible", &Node::is_feasible)
      .def("get_fixed_sequence", &Node::get_fixed_sequence);

  py::class_<SolutionPool>(m, "SolutionPool")
      .def("add_solution", &SolutionPool::add_solution)
      .def("get_upper_bound", &SolutionPool::get_upper_bound)
      .def("empty", &SolutionPool::empty)
      .def("get_best_solution", &SolutionPool::get_best_solution);

  py::class_<Instance>(m, "Instance", "CE-TSP Instance")
      .def(py::init<std::vector<Circle>>())
      .def(py::init(
          [](std::vector<Circle> &circles, Point &a, Point &b) -> Instance {
            Instance instance{circles};
            instance.path = {a, b};
            return instance;
          }))
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
  py::class_<EventContext>(m, "EventContext")
      .def_readonly("current_node", &EventContext::current_node,
                    py::return_value_policy::reference)
      .def_readonly("root_node", &EventContext::root_node,
                    py::return_value_policy::reference)
      .def_readonly("instance", &EventContext::instance,
                    py::return_value_policy::reference)
      .def_readonly("num_iterations", &EventContext::num_iterations)
      .def("add_lazy_circle", &EventContext::add_lazy_circle)
      .def("add_solution", &EventContext::add_solution)
      .def("get_lower_bound", &EventContext::get_lower_bound)
      .def("get_upper_bound", &EventContext::get_upper_bound)
      .def("is_feasible", &EventContext::is_feasible)
      .def("get_relaxed_solution", &EventContext::get_relaxed_solution)
      .def("get_best_solution", &EventContext::get_best_solution);
  m.def("branch_and_bound", &branch_and_bound);
}