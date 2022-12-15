//
// Created by Dominik Krupke on 12.12.22.
//

#ifndef CETSP_BNB_H
#define CETSP_BNB_H
#include "branching_strategy.h"
#include "node.h"
#include "root_node_strategy.h"
#include "search_strategy.h"
#include "solution_pool.h"
namespace cetsp {

class NodeProcessingStrategy {
public:
  void process(Node &node) {}
};

class BranchAndBoundAlgorithm {
public:
  BranchAndBoundAlgorithm(const std::vector<Circle> *instance)
      : root_node_strategy{}, root{root_node_strategy.get_root_node(*instance,
                                                                    false)},
        search_strategy(root), branching_strategy(instance) {}

  void add_upper_bound(const Trajectory &trajectory) {
    solution_pool.add_solution(trajectory);
  }
  void add_lower_bound(double lb) { root.add_lower_bound(lb); }

  double get_upper_bound() { return solution_pool.get_upper_bound(); }
  double get_lower_bound() { return root.get_lower_bound(); }
  std::optional<Trajectory> get_solution() {
    if (solution_pool.empty()) {
      return {};
    }
    return solution_pool.get_best_solution();
  }

  void optimize(int timelimit_s, double gap = 0.01, bool verbose = true) {
    if (verbose) {
      std::cout << "LB\t|\tUB" << std::endl;
    }
    while (step()) {
      auto lb = get_lower_bound();
      auto ub = get_upper_bound();
      if (verbose) {
        std::cout << lb << "\t|\t" << ub << std::endl;
      }
      if (ub <= (1 + gap) * lb) {
        return;
      }
    }
  }

private:
  bool step() {
    Node *node = search_strategy.next();

    if (node == nullptr) {
      return false;
    }
    if (node->is_pruned() ||
        node->get_lower_bound() >= solution_pool.get_upper_bound()) {
      node->prune();
      return true;
    }
    node_processing_strategy.process(*node);
    if (node->is_pruned()) {
      return true;
    }
    if (node->is_feasible(0.01)) {
      solution_pool.add_solution(node->get_relaxed_solution());
      return true;
    } else {
      // branch
      if (branching_strategy.branch(*node)) {
        search_strategy.notify_of_branch(*node);
      }

      return true;
    }
    assert(false); // unreachable
  }

  std::vector<Circle> instance;
  RootNodeStrategy root_node_strategy;
  Node root;
  SearchStrategy search_strategy;
  NodeProcessingStrategy node_processing_strategy;
  BranchingStrategy branching_strategy;
  SolutionPool solution_pool;
};

TEST_CASE("Branch and Bound  1") {
  std::vector<Circle> instance = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  BranchAndBoundAlgorithm bnb(&instance);
  bnb.optimize(30);
}

TEST_CASE("Branch and Bound  2") {
  std::vector<Circle> instance = {
      {{0, 0}, 0.0}, {{5, 0}, 0.0}, {{5, 5}, 0.0}, {{0, 5}, 0.0}};
  BranchAndBoundAlgorithm bnb(&instance);
  bnb.optimize(30);
  CHECK(bnb.get_upper_bound() == doctest::Approx(20));
}
} // namespace cetsp
#endif // CETSP_BNB_H
