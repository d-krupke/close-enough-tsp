#pragma once

#include "node.hpp"
#include "relaxation.hpp"
#include "relaxation_solver.hpp"

namespace cetsp_solver {

class SearchWorker {
public:
  SearchWorker(Instance &instance, NodeFactory &node_factory,
               SearchManager &search_manager, SolutionPool &solution_pool)
      : instance{instance}, node_factory(node_factory),
        relaxation_solver(&instance), search_manager(search_manager),
        solution_pool(solution_pool) {}

  void run() {
    while (true) {
      auto node = search_manager.get_next_node(min_lb);
      if (node == nullptr) {
        break;
      }
      if (!process_node(*node)) {
        search_manager.return_node(node);
      }
    }
  }

  bool process_node(Node &node) {
    // Return true if the node is completely processed, false otherwise

    // Check if the trivial lower bound is already worse than the best known
    auto current_ub = solution_pool.get_best_cost();
    if (check_bound_and_prune(node, current_ub)) {
      return true;
    }
    switch (node.status) {
    case NodeStatus::UNKOWN:
      return preprocess_node(node) || check_bound_and_prune(node, current_ub);
    case NodeStatus::PREPROCESSED:
      return relax_node(node) || check_bound_and_prune(node, current_ub);
    case NodeStatus::RELAXED:
      return evaluate_relaxation_of_node(node) ||
             check_bound_and_prune(node, current_ub);
    case NodeStatus::INCOMPLETE:
      return branch_node(node);
    default:
      throw std::runtime_error("Unexpected node status");
    }
  }

protected:
  bool preprocess_node(Node &node) {
    // Add potential preprocessing steps here
    // ...
    node.status = NodeStatus::PREPROCESSED;
    return false;
  }

  bool relax_node(Node &node) {
    // Solve the relaxation of the node
    relaxation_solver.process_node(node);
    return false;
  }

  bool evaluate_relaxation_of_node(Node &node) {
    // Evaluate the node
    assert(node.trajectory.has_value());
    if (!node.annotated_trajectory.has_value()) {
      node.annotated_trajectory =
          AnnotatedRelaxedSolution(&instance, node.trajectory.value());
    }
    if (node.annotated_trajectory->is_feasible()) {
      Solution solution(node.trajectory->first, node.trajectory->second);
      solution_pool.add_solution(solution);
      node.status = NodeStatus::FEASIBLE;
      search_manager.remove_node(&node, true);
    } else {
      node.status = NodeStatus::INCOMPLETE;
      return false;
    }
    return false;
  }

  bool check_bound_and_prune(Node &node, double current_ub) {
    if (node.lb >= current_ub) {
      // We want to keep the lower bound of the node as it is not infeasible
      // Otherwise, we would get a lower bound of infinity if a heuristic
      // provides the optimal solution and all solutions in the search tree
      // are pruned.
      node.status = NodeStatus::PRUNED;
      search_manager.remove_node(&node, /*keep_lower_bound=*/true);
      return true;
    }
    return false;
  }

  bool branch_node(Node &node) {
    auto [idx, dist] = node.annotated_trajectory->get_max_distance();
    // place the circle with the index at every possible position in the
    // sequence
    for (uint64_t i = 1; i <= node.sequence.size(); i++) {
      auto new_sequence = node.sequence;
      new_sequence.insert(new_sequence.begin() + i, idx);
      auto new_node = node_factory.create_child_node(node, new_sequence);
      search_manager.add_node(std::move(new_node));
    }
    node.status = NodeStatus::BRANCHED;
    search_manager.remove_node(&node, /*keep_lower_bound=*/false);
    return true;
  }

private:
  Instance &instance;
  NodeFactory &node_factory;
  SocpRelaxationSolver relaxation_solver;
  SearchManager &search_manager;
  SolutionPool &solution_pool;
};
} // namespace cetsp_solver