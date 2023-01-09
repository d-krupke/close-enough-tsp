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
  BranchAndBoundAlgorithm(Instance *instance, Node root_,
                          BranchingStrategy &branching_strategy,
                          SearchStrategy &search_strategy,
                          UserCallbacks user_callbacks = DefaultUserCallbacks())
      : instance{instance}, root{std::move(root_)},
        search_strategy{search_strategy}, user_callbacks{user_callbacks},
        branching_strategy(branching_strategy) {
    branching_strategy.setup(instance, &root, &solution_pool);
    search_strategy.init(root);
  }

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
      std::cout << "Starting with root node of size "
                << root.get_fixed_sequence().size() << std::endl;
      std::cout << "i\tLB\t|\tUB\t|\tTime" << std::endl;
    }
    using namespace std::chrono;
    auto start = high_resolution_clock::now();
    while (step(gap)) {
      auto lb = get_lower_bound();
      auto ub = get_upper_bound();
      auto now = high_resolution_clock::now();
      const auto time_used  = duration_cast<seconds>(now - start).count();
      if (verbose) {
        if (num_iterations <= 10 ||
            (num_iterations < 100 && num_iterations % 10 == 0) ||
            (num_iterations < 1000 && num_iterations % 100 == 0) ||
            (num_iterations % 1000 == 0)) {
          std::cout << num_iterations << "\t" << lb << "\t|\t" << ub
                    <<"\t|\t"<< time_used <<"s"<< std::endl;
        }
      }
      if (ub <= (1 + gap) * lb) {
        break;
      }
      ++num_iterations;

      if (time_used > timelimit_s) {
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
      std::cout << num_steps << " iterations with "<<num_explored<<" nodes explored and "<<num_branches<<" branches." << std::endl;
    }
  }

private:
  bool prune_if_above_ub(Node* node, const double gap) {
    if (node->is_pruned() ||
        node->get_lower_bound() >= (1.0-gap)*solution_pool.get_upper_bound()) {
      node->prune();
      on_prune(*node);
      return true;
    }
    return  false;
  }

  /**
   * Executes a step/node exploration in the BnB-algorithm.
   * @return
   */
  bool step(double gap) {
    num_steps +=  1;
    Node *node = search_strategy.next();

    // No further node to explore.
    if (node == nullptr) { return false; }
    // Automatically prune if worse than upper bound.
    if(prune_if_above_ub(node, gap)) { return true; }
    // Explore  node.
    num_explored +=  1;
    EventContext context{node, &root, instance, &solution_pool, num_iterations};
    user_callbacks.on_entering_node(context);
    if(!node->is_pruned()) {
      explore_node(node, context, gap);
    }
    on_leaving_node(context);
    return true;
  }

  void explore_node(Node* node, EventContext& context, const double gap){
    if (node->is_feasible()) {
      // If node is  feasible, check lazy constraints.
      user_callbacks.add_lazy_constraints(context);
    }
    if (node->is_feasible()) { // this can have changed after lazy callbacks.
      solution_pool.add_solution(node->get_relaxed_solution());
      on_feasible(context);
    } else {
      // Check again for the bound before branching.
      if(prune_if_above_ub(node, gap)) { return; }
      // branch if not yet feasible.
      if (branching_strategy.branch(*node)) {
        num_branches += 1;
        search_strategy.notify_of_branch(*node);
      }
    }
  }

  void on_prune(Node &node) { search_strategy.notify_of_prune(node); }

  void on_feasible(EventContext &context) {

    //
    search_strategy.notify_of_feasible(*(context.current_node));
  }

  void on_leaving_node(EventContext &context) {
    user_callbacks.on_leaving_node(context);
  }

  Instance *instance;

  Node root;
  SearchStrategy &search_strategy;
  UserCallbacks user_callbacks;
  BranchingStrategy &branching_strategy;
  SolutionPool solution_pool;
  int num_iterations = 0;
  int num_steps = 0;
  int num_explored = 0;
  int num_branches = 0;
};

TEST_CASE("Branch and Bound  1") {
  Instance instance({{{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}});
  CHECK(instance.size() == 4);
  LongestEdgePlusFurthestCircle root_node_strategy{};
  FarthestCircle branching_strategy;
  CheapestChildDepthFirst search_strategy;
  BranchAndBoundAlgorithm bnb(&instance,
                              root_node_strategy.get_root_node(instance),
                              branching_strategy, search_strategy);
  bnb.optimize(30);
}

TEST_CASE("Branch and Bound  2") {
  Instance instance(
      {{{0, 0}, 0.0}, {{5, 0}, 0.0}, {{5, 5}, 0.0}, {{0, 5}, 0.0}});
  LongestEdgePlusFurthestCircle root_node_strategy{};
  FarthestCircle branching_strategy;
  CheapestChildDepthFirst search_strategy;
  BranchAndBoundAlgorithm bnb(&instance,
                              root_node_strategy.get_root_node(instance),
                              branching_strategy, search_strategy);
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
  LongestEdgePlusFurthestCircle root_node_strategy{};
  FarthestCircle branching_strategy;
  CheapestChildDepthFirst search_strategy;
  BranchAndBoundAlgorithm bnb(&instance,
                              root_node_strategy.get_root_node(instance),
                              branching_strategy, search_strategy);
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
  LongestEdgePlusFurthestCircle root_node_strategy{};
  FarthestCircle branching_strategy;
  CheapestChildDepthFirst search_strategy;
  BranchAndBoundAlgorithm bnb(&instance,
                              root_node_strategy.get_root_node(instance),
                              branching_strategy, search_strategy);
  bnb.optimize(30);
  CHECK(bnb.get_solution());
  CHECK(bnb.get_upper_bound() == doctest::Approx(42.0747));
}
} // namespace cetsp
#endif // CETSP_BNB_H
