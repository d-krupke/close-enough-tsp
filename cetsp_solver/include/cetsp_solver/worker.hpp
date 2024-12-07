// This file defines a search worker that will process nodes from the
// branch-and-bound tree in parallel. The process can be divided into several
// stages, as processing a node can be expensive and we might want to switch to
// a more promising node in between based on changes in the bounds.

#pragma once

#include "branching_strategy.hpp"
#include "node.hpp"
#include "relaxation.hpp"
#include "relaxation_solver.hpp"
#include <atomic>
#include <chrono>
#include <thread>

namespace cetsp_solver {

class SearchWorker {
public:
  SearchWorker(Instance &instance, NodeFactory &node_factory,
               SearchManager &search_manager, SolutionPool &solution_pool)
      : instance{instance}, node_factory(node_factory),
        relaxation_solver(&instance),
        branch_strategy(node_factory, search_manager),
        search_manager(search_manager), solution_pool(solution_pool),
        stop_flag(false) {}

  void run() {
    while (!stop_flag.load()) {
      auto node = search_manager.get_next_node(min_lb);
      if (node == nullptr) {
        if (search_manager.is_empty()) {
          break; // terminate
        }
        // sleep for 20ms to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      if (!process_node(*node)) {
        // return the unfinished node to the search manager
        // We may get it back in the next iteration, but we may also get
        // a different node that now is more promising.
        search_manager.requeue_node(node);
      }
    }
  }

  bool process_node(Node &node) {
    // Return true if the node is completely processed, false otherwise

    // Check if the trivial lower bound is already worse than the best known
    const auto current_ub = solution_pool.get_best_cost();
    if (check_bound_and_prune(node, current_ub)) {
      return true;
    }
    switch (node.status) {
    case NodeStatus::UNKNOWN:
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

  void stop() { stop_flag.store(true); }

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
      search_manager.close_node(&node, true);
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
      search_manager.close_node(&node, /*keep_lower_bound=*/true);
      return true;
    }
    return false;
  }

  bool branch_node(Node &node) {
    branch_strategy.branch_node(node);
    return true;
  }

private:
  Instance &instance;
  NodeFactory &node_factory;
  SocpRelaxationSolver relaxation_solver;
  BranchingStrategy branch_strategy;
  SearchManager &search_manager;
  SolutionPool &solution_pool;
  std::atomic<bool> stop_flag;
};
} // namespace cetsp_solver