/**
 * This file implements branching stratgies for the branch and bound algorithm,
 * i.e., deciding how to split  the solution space. The primary decision here
 * is to decide for the circle to integrate next. However, we can also do
 * some filtering here and only create branches  that are promising.
 *
 * The simplest branching strategy is to use always the furthest circle to
 * the current relaxed solution.
 */
#ifndef CETSP_BRANCHING_STRATEGY_H
#define CETSP_BRANCHING_STRATEGY_H
#include "cetsp/common.h"
#include "cetsp/details/convex_hull_order.h"
#include "cetsp/details/solution_pool.h"
#include "cetsp/details/triple_map.h"
#include "cetsp/node.h"
#include "rule.h"
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>
#include <random>
#include <vector>
namespace cetsp {

class BranchingStrategy {
public:
  virtual void setup(Instance *instance, std::shared_ptr<Node> &root,
                     SolutionPool *solution_pool) {}
  virtual bool branch(Node &node) = 0;
  virtual ~BranchingStrategy() = default;
};

class CircleBranching : public BranchingStrategy {
public:
  explicit CircleBranching(bool simplify = false, size_t num_threads = 1)
      : simplify{simplify}, num_threads{num_threads} {
    if (simplify) {
      std::cout << "Using node simplification." << std::endl;
    }
    std::cout << "Exploring on " << num_threads << " threads" << std::endl;
  }

  void setup(Instance *instance_, std::shared_ptr<Node> &root,
             SolutionPool *solution_pool) override {
    instance = instance_;
    for (auto &rule : rules) {
      rule->setup(instance, root, solution_pool);
    }
  }

  void add_rule(std::unique_ptr<SequenceRule> &&rule) {
    rules.push_back(std::move(rule));
  }

  bool branch(Node &node) override;

protected:
  /**
   * Override this method to filter the branching in advance.
   * @param sequence Sequence to be checked for a potential branch.
   * @return True if branch should be created.
   */
  virtual bool is_sequence_ok(const std::vector<int> &sequence) {
    return std::all_of(rules.begin(), rules.end(), [&sequence](auto &rule) {
      return rule->is_ok(sequence);
    });
  }

  virtual std::optional<int> get_branching_circle(Node &node) = 0;

  Instance *instance = nullptr;
  bool simplify;
  size_t num_threads;
  std::vector<std::unique_ptr<SequenceRule>> rules;
};

class FarthestCircle : public CircleBranching {
  /**
   * This strategy tries to branch on the circle that is most distanced
   * to the relaxed solution.
   */
public:
  explicit FarthestCircle(bool simplify = false, size_t num_threads = 1)
      : CircleBranching{simplify, num_threads} {
    std::cout << "Branching on farthest circle." << std::endl;
  }

protected:
  std::optional<int> get_branching_circle(Node &node) override;
};

class ChFarthestCircle : public FarthestCircle {
  /**
   * This strategy will only create branches that satisfy the CCW order of the
   * convex hull. A dependency is that the root is also obeying this rule.
   * We can proof that any optimal solution has follow the order of circles
   * intersecting the convex hull on the circle centers.
   *
   * This does not allow lazy constraints! (or only those that do not change
   * the convex hull).
   */
public:
  explicit ChFarthestCircle(bool simplify = true, size_t num_threads = 1);
};

class RandomCircle : public CircleBranching {
  /**
   * Just a random branching  strategy as comparison. It will branch on a random
   * not yet covered circle.
   */
public:
  explicit RandomCircle(bool simplify = false, size_t num_threads = 1)
      : CircleBranching{simplify, num_threads} {
    std::cout << "Branching on random circle" << std::endl;
  }

protected:
  std::optional<int> get_branching_circle(Node &node) override;
};

TEST_CASE("Branching Strategy") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  std::vector<Circle> instance_ = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  Instance instance(instance_);
  FarthestCircle bs(false);
  auto root = std::make_shared<Node>(std::vector<int>{0, 1, 2, 3}, &instance);
  bs.setup(&instance, root, nullptr);
  CHECK(bs.branch(*root) == false);

  Node root2({0, 1, 2}, &instance);
  CHECK(bs.branch(root2) == true);
  CHECK(root2.get_children().size() == 3);
}

} // namespace cetsp
#endif // CETSP_BRANCHING_STRATEGY_H
