/**
 * This file defines the python bindings.
 */
#include "cetsp/bnb.h"
#include "cetsp/common.h"
#include "cetsp/details/cross_lower_bound.h"
#include "cetsp/details/triple_map.h"
#include "cetsp/heuristics.h"
#include "cetsp/node.h"
#include "cetsp/strategies/rules/global_convex_hull_rule.h"
#include "cetsp/strategies/rules/layered_convex_hull_rule.h"
#include <fmt/core.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h> // to define operator overloading
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // automatic conversion of vectors
namespace py = pybind11;
using namespace cetsp;
using namespace cetsp::details;

class PythonCallback : public B2BNodeCallback {
  /*
   * Allowing user-callbacks to influence/improve the branch
   * and bound algorithm.
   */
public:
  PythonCallback(std::function<void(EventContext)> *callback)
      : callback(callback) {}

  void on_entering_node(EventContext &e) {
    EventContext e_ = e; // making sure, the callback will get a copy and
                         // there won't be any accidental ownership problems.
    assert(callback != nullptr);
    (*callback)(e_);
  }

private:
  std::function<void(EventContext)> *callback;
};

/**
 * Explicit function for the binding  of calling the BnB algorithm.
 * @param instance Instance to be solved.
 * @param py_callback Callback function in Python.
 * @param initial_solution Initial solution to get started.
 * @param timelimit Timelimit in seconds for the BnB algorithm.
 * @return Best solution found within timelimit or nullptr.
 */
std::tuple<std::unique_ptr<Solution>, double,
           std::unordered_map<std::string, std::string>>
branch_and_bound(Instance instance,
                 std::function<void(EventContext)> *py_callback,
                 Solution *initial_solution, int timelimit,
                 std::string branching, std::string search, std::string root,
                 std::vector<std::string> rules, size_t num_threads) {

  std::unique_ptr<RootNodeStrategy> rns;
  if (root == "ConvexHull") {
    rns = std::make_unique<ConvexHullRoot>();
  } else if (root == "LongestEdgePlusFurthestCircle") {
    rns = std::make_unique<LongestEdgePlusFurthestCircle>();
  } else if (root == "Random") {
    rns = std::make_unique<RandomRoot>();
  } else {
    throw std::invalid_argument("Invalid root node strategy");
  }
  std::unique_ptr<CircleBranching> branching_strategy;
  if (branching == "FarthestCircle") {
    branching_strategy = std::make_unique<FarthestCircle>(false, num_threads);
  } else if (branching == "ChFarthestCircle") {
    branching_strategy = std::make_unique<ChFarthestCircle>(false, num_threads);
  } else if (branching == "ChFarthestCircleSimplifying") {
    branching_strategy = std::make_unique<ChFarthestCircle>(true, num_threads);
  } else if (branching == "Random") {
    branching_strategy = std::make_unique<RandomCircle>(true, num_threads);
  } else {
    throw std::invalid_argument("Invalid branching strategy.");
  }
  std::unique_ptr<SearchStrategy> search_strategy;
  if (search == "DfsBfs") {
    search_strategy = std::make_unique<DfsBfs>();
  } else if (search == "CheapestChildDepthFirst") {
    search_strategy = std::make_unique<CheapestChildDepthFirst>();
  } else if (search == "CheapestBreadthFirst") {
    search_strategy = std::make_unique<CheapestBreadthFirst>();
  } else if (search == "Random") {
    search_strategy = std::make_unique<RandomNextNode>();
  } else {
    throw std::invalid_argument("Invalid search strategy.");
  }

  std::unordered_set<std::string> rules_(rules.begin(), rules.end());
  for (const auto &rule_name : rules_) {
    if (rule_name == "GlobalConvexHullRule") {
      branching_strategy->add_rule(std::make_unique<GlobalConvexHullRule>());
    } else if (rule_name == "LayeredConvexHullRule") {
      branching_strategy->add_rule(std::make_unique<LayeredConvexHullRule>());
    } else {
      throw std::invalid_argument("Invalid rule.");
    }
  }

  BranchAndBoundAlgorithm baba(&instance, rns->get_root_node(instance),
                               *branching_strategy, *search_strategy);
  /* TODO add CrossLowerBoundCallback according to some config */
  // baba.add_node_callback(std::make_unique<CrossLowerBoundCallback>());
  baba.add_node_callback(std::make_unique<PythonCallback>(py_callback));

  if (initial_solution != nullptr) {
    baba.add_upper_bound(*initial_solution);
  }
  baba.optimize(timelimit);
  return {baba.get_solution(), baba.get_lower_bound(), baba.get_statistics()};
}

PYBIND11_MODULE(_cetsp_bindings, m) {
  // Classes
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
      .def("contains",
           [](const Circle &self, const Point &p) { return self.contains(p); })
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
      .def("distance", &Trajectory::distance)
      .def("is_simple", &Trajectory::is_simple);

  py::class_<Node>(m, "Node", "Node in the BnB-tree.")
      .def("depth", &Node::depth)
      .def("get_lower_bound", &Node::get_lower_bound)
      .def("get_relaxed_solution", &Node::get_relaxed_solution)
      .def("add_lower_bound", &Node::add_lower_bound)
      .def("prune", &Node::prune)
      .def("is_pruned", &Node::is_pruned)
      .def("is_feasible", &Node::is_feasible)
      .def("get_fixed_sequence", &Node::get_fixed_sequence)
      .def("get_spanning_sequence", &Node::get_spanning_sequence);

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
      .def("__len__", [](const Instance &self) { return self.size(); })
      .def("__getitem__",
           [](const Instance &self, int i) { return self.at(i); })
      .def("circles", [](const Instance &self) {
        std::vector<Circle> circles;
        for (const auto &c : self) {
          circles.push_back(c);
        }
        return circles;
      });
  /**
   * This is the probably most interesting class for the bindings.
   * It allows you to manipulate the BnB-process via a callback.
   */
  py::class_<EventContext>(
      m, "EventContext",
      "Allows you to extract information of the BnB process and influence it.")
      .def_readonly("current_node", &EventContext::current_node,
                    py::return_value_policy::reference,
                    "The node currently investigated.")
      .def_readonly("root_node", &EventContext::root_node,
                    py::return_value_policy::reference,
                    "The root node of the BnB tree.")
      .def_readonly("instance", &EventContext::instance,
                    py::return_value_policy::reference,
                    "The instance solved by the BnB tree.")
      .def_readonly("num_iterations", &EventContext::num_iterations,
                    "Number of iterations/nodes visited in the BnB tree.")
      .def("add_lazy_circle", &EventContext::add_lazy_circle,
           "Add a circle to the instance as kind of a lazy constraint. Must be "
           "deterministic.")
      .def("add_solution", &EventContext::add_solution,
           "Add a new solution, that may helps terminating earlier.")
      .def("get_lower_bound", &EventContext::get_lower_bound,
           "Return the currently proven lower bound.")
      .def("get_upper_bound", &EventContext::get_upper_bound,
           "Return  the value of the currently best known solution (or "
           "infinity).")
      .def("is_feasible", &EventContext::is_feasible,
           "Return true if the current node is feasible.")
      .def("get_relaxed_solution", &EventContext::get_relaxed_solution,
           "Return the relaxed solution of the current node.")
      .def("get_best_solution", &EventContext::get_best_solution,
           "Return the best known feasible solution.");
  py::class_<TripleMap>(m, "TripleMap", "bla")
      .def(py::init<Instance *>())
      .def("get_cost", &TripleMap::get_cost);
  py::class_<PartialSequenceSolution>(m, "PartialSequenceSolution")
      .def("get_trajectory", &PartialSequenceSolution::get_trajectory);
  py::class_<Solution>(m, "Solution")
      .def("get_trajectory", &Solution::get_trajectory);

  // functions
  m.def("compute_tour_by_2opt", &compute_tour_by_2opt,
        "Compute a feasible tour by a two-opt technique.");
  m.def("compute_tour_from_sequence",
        py::overload_cast<const std::vector<Circle> &, bool>(&compute_tour),
        "Computes a close-enough tour based on a given circle sequence.");
  m.def("branch_and_bound", &branch_and_bound,
        "Computes an optimal solution based on BnB.", py::arg("instance"),
        py::arg("callback"), py::arg("initial_solution") = nullptr,
        py::arg("timelimit") = 300,
        py::arg("branching") = "ChFarthestCircleSimplifying",
        py::arg("search") = "DfsBfs", py::arg("root") = "ConvexHull",
        py::arg("rules") = std::vector<std::string>{"GlobalConvexHullRule"},
        py::arg("num_threads") = 8);
}
