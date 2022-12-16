//
// Created by Dominik Krupke on 16.12.22.
//

#ifndef CETSP_CALLBACKS_H
#define CETSP_CALLBACKS_H
#include "cetsp/details/branching_strategy.h"
#include "cetsp/details/root_node_strategy.h"
#include "cetsp/details/search_strategy.h"
#include "cetsp/details/solution_pool.h"
namespace cetsp {

struct EventContext {
  Node *current_node;
  Node *root_node;
  Instance *instance;
  SolutionPool *solution_pool;
  int num_iterations;

  void add_lazy_circle(Circle &circle) { instance->add_circle(circle); }

  void add_solution(Trajectory &trajectory) {
    solution_pool->add_solution(trajectory);
  }

  double get_lower_bound() { return root_node->get_lower_bound(); }

  double get_upper_bound() { return solution_pool->get_upper_bound(); }

  bool is_feasible() { return current_node->is_feasible(); }

  Trajectory get_relaxed_solution() {
    return current_node->get_relaxed_solution();
  }

  std::unique_ptr<Trajectory> get_best_solution() {
    return solution_pool->get_best_solution();
  }
};

class DefaultUserCallbacks {
public:
  void on_entering_node(EventContext &e) {}
  void add_lazy_constraints(EventContext &e) {}
  void on_leaving_node(EventContext &e) {}
};
};     // namespace cetsp
#endif // CETSP_CALLBACKS_H
