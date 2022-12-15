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
#include <chrono>

namespace cetsp {

class NodeProcessingStrategy {
public:
  void process(Node &node, SolutionPool& solution_pool) {}
};

template <typename TNodeProcessingStrategy = NodeProcessingStrategy>
class BranchAndBoundAlgorithm {
public:
  BranchAndBoundAlgorithm(const Instance *instance, TNodeProcessingStrategy node_processing_strategy=NodeProcessingStrategy())
      : root_node_strategy{}, root{root_node_strategy.get_root_node(*instance)}, node_processing_strategy{node_processing_strategy},
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

  void optimize(int timelimit_s, double gap = 0.01, double eps = 0.01,
                bool verbose = true) {
    if (verbose) {
      std::cout << "i\tLB\t|\tUB" << std::endl;
    }
    int i = 0;
    using namespace std::chrono;
    auto start = high_resolution_clock::now();
    while (step(eps)) {
      auto lb = get_lower_bound();
      auto ub = get_upper_bound();
      if (verbose) {
        if (i <= 10 || (i < 100 && i % 10 == 0) || (i % 100 == 0)) {
          std::cout << i << "\t" << lb << "\t|\t" << ub << std::endl;
        }
      }
      if (ub <= (1 + gap) * lb) {
        break;
      }
      ++i;
      auto now = high_resolution_clock::now();
      if(duration_cast<seconds>(now - start).count()>timelimit_s){
        if(verbose) {
          std::cout << "Timeout." << std::endl;
        }
        break;
      }
    }
    if (verbose) {
      auto lb = get_lower_bound();
      auto ub = get_upper_bound();
      std::cout << "---------------" << std::endl
                << i << "\t" << lb << "\t|\t" << ub << std::endl;
    }
  }

private:
  bool step(double eps) {
    Node *node = search_strategy.next();

    if (node == nullptr) {
      return false;
    }
    if (node->is_pruned() ||
        node->get_lower_bound() >= solution_pool.get_upper_bound()) {
      node->prune();
      return true;
    }
    node_processing_strategy.process(*node, solution_pool);
    if (node->is_pruned()) {
      return true;
    }
    if (node->is_feasible(eps)) {
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

  RootNodeStrategy root_node_strategy;
  Node root;
  SearchStrategy search_strategy;
  TNodeProcessingStrategy node_processing_strategy;
  BranchingStrategy branching_strategy;
  SolutionPool solution_pool;
};

TEST_CASE("Branch and Bound  1") {
  Instance instance({{{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}});
  CHECK(instance.size() == 4);
  BranchAndBoundAlgorithm bnb(&instance);
  bnb.optimize(30);
}

TEST_CASE("Branch and Bound  2") {
  Instance instance(
      {{{0, 0}, 0.0}, {{5, 0}, 0.0}, {{5, 5}, 0.0}, {{0, 5}, 0.0}});
  BranchAndBoundAlgorithm bnb(&instance);
  bnb.optimize(30);
  CHECK(bnb.get_upper_bound() == doctest::Approx(20));
}

TEST_CASE("Branch and Bound  3") {
  Instance instance;
  for (double x = 0; x <= 10; x += 2.0) {
    for (double y = 0; y <= 10; y += 2.0) {
      instance.push_back({{x, y}, 1});
    }
  }
  BranchAndBoundAlgorithm bnb(&instance);
  bnb.optimize(30);
  CHECK(bnb.get_upper_bound() <= 41);
}

TEST_CASE("Branch and Bound Path") {
  Instance instance;
  for (double x = 0; x <= 10; x += 2.0) {
    for (double y = 0; y <= 10; y += 2.0) {
      instance.push_back({{x, y}, 1});
    }
  }
  instance.path = {{0,0}, {0,0}};
  BranchAndBoundAlgorithm bnb(&instance);
  bnb.optimize(30);
  CHECK(bnb.get_upper_bound() == doctest::Approx(42.0747));
}
} // namespace cetsp
#endif // CETSP_BNB_H
