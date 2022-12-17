/**
 * This file implements the Branch and Bound algorithm. The sub-strategies
 * are separated in other classes to make it easily adaptable.
 * With the user callbacks, you can influence it strongly.
 */

#ifndef CETSP_BNB_H
#define CETSP_BNB_H
#include "cetsp/callbacks.h"
#include "cetsp/details/branching_strategy.h"
#include "cetsp/details/root_node_strategy.h"
#include "cetsp/details/search_strategy.h"
#include "cetsp/details/solution_pool.h"
#include "node.h"
#include <chrono>
namespace cetsp {

template <typename UserCallbacks = DefaultUserCallbacks>
class BranchAndBoundAlgorithm {
  /**
   * Implements the branch and bound algorithm.
   * TODO: Make the strategies replaceable by templates.
   */
public:
  BranchAndBoundAlgorithm(Instance *instance,
                          UserCallbacks user_callbacks = DefaultUserCallbacks())
      : instance{instance}, root{root_node_strategy.get_root_node(*instance)},
         search_strategy(root),user_callbacks{user_callbacks},
        branching_strategy(instance) {}

  /**
   * Add a feasible solution as upper bound. Note that it must also obey
   * all lazy constraints. This can speed up the algorithm as it can allow BnB
   * to prune a lot of suboptimal branches. You can add as many solutions as
   * you want as only the best is used.
   * @param trajectory The feasible solution.
   */
  void add_upper_bound(const Trajectory &trajectory) {
    solution_pool.add_solution(trajectory);
  }

  /**
   * Add a lower bound to the BnB-tree. This usually does not help much, it
   * may only be a benefit, it is higher than any LB found by BnB, but it does
   * probably not influence the algorithm itself as long as it is not very close
   * to the optimum.
   * @param lb The lower bound.
   */
  void add_lower_bound(double lb) { root.add_lower_bound(lb); }

  double get_upper_bound() { return solution_pool.get_upper_bound(); }
  double get_lower_bound() { return root.get_lower_bound(); }
  std::unique_ptr<Trajectory> get_solution() {
    return solution_pool.get_best_solution();
  }

  /**
   * Run the Branch and Bound algorithm.
   * @param timelimit_s The timelimit in seconds, after which it aborts.
   * @param gap Allowed optimality gap.
   * @param verbose Defines if you want to see a progress log.
   */
  void optimize(int timelimit_s, double gap = 0.01, bool verbose = true) {
    if (verbose) {
      std::cout << "i\tLB\t|\tUB" << std::endl;
    }
    using namespace std::chrono;
    auto start = high_resolution_clock::now();
    while (step()) {
      auto lb = get_lower_bound();
      auto ub = get_upper_bound();
      if (verbose) {
        if (num_iterations <= 10 ||
            (num_iterations < 100 && num_iterations % 10 == 0) ||
            (num_iterations % 100 == 0)) {
          std::cout << num_iterations << "\t" << lb << "\t|\t" << ub
                    << std::endl;
        }
      }
      if (ub <= (1 + gap) * lb) {
        break;
      }
      ++num_iterations;
      auto now = high_resolution_clock::now();
      if (duration_cast<seconds>(now - start).count() > timelimit_s) {
        if (verbose) {
          std::cout << "Timeout." << std::endl;
        }
        break;
      }
    }
    if (verbose) {
      auto lb = get_lower_bound();
      auto ub = get_upper_bound();
      std::cout << "---------------" << std::endl
                << num_iterations << "\t" << lb << "\t|\t" << ub << std::endl;
    }
  }

private:
  /**
   * Executes a step/node exploration in the BnB-algorithm.
   * @return
   */
  bool step() {
    Node *node = search_strategy.next();

    // No further node to explore.
    if (node == nullptr) {
      return false;
    }
    // Automatically prune if worse than upper bound.
    if (node->is_pruned() ||
        node->get_lower_bound() >= solution_pool.get_upper_bound()) {
      node->prune();
      return true;
    }
    // Explore  node.
    EventContext context{node, &root, instance, &solution_pool, num_iterations};
    user_callbacks.on_entering_node(context);
    if (node->is_pruned()) {
      user_callbacks.on_leaving_node(context);
      return true;
    }
    if (node->is_feasible()) {
      // If node is  feasible, check lazy constraints.
      user_callbacks.add_lazy_constraints(context);
    }
    if (node->is_feasible()) { // this can have changed after lazy callbacks.
      solution_pool.add_solution(node->get_relaxed_solution());
    } else {
      // branch if not yet feasible.
      if (branching_strategy.branch(*node)) {
        search_strategy.notify_of_branch(*node);
      }
    }
    user_callbacks.on_leaving_node(context);
    return true;
  }

  Instance *instance;
  LongestEdgePlusFurthestCircle root_node_strategy{};
  Node root;
  SearchStrategy search_strategy;
  UserCallbacks user_callbacks;
  BranchingStrategy branching_strategy;
  SolutionPool solution_pool;
  int num_iterations = 0;
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
  CHECK(bnb.get_solution());
  CHECK(bnb.get_solution()->length() == doctest::Approx(20));
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
  CHECK(bnb.get_solution());
  CHECK(bnb.get_upper_bound() <= 41);
}

TEST_CASE("Branch and Bound Path") {
  Instance instance;
  for (double x = 0; x <= 10; x += 2.0) {
    for (double y = 0; y <= 10; y += 2.0) {
      instance.push_back({{x, y}, 1});
    }
  }
  instance.path = {{0, 0}, {0, 0}};
  BranchAndBoundAlgorithm bnb(&instance);
  bnb.optimize(30);
  CHECK(bnb.get_solution());
  CHECK(bnb.get_upper_bound() == doctest::Approx(42.0747));
}
} // namespace cetsp
#endif // CETSP_BNB_H
