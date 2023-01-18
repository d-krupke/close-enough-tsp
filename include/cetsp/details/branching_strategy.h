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
#include "cetsp/details/solution_pool.h"
#include "cetsp/details/triple_map.h"
#include "cetsp/node.h"
#include "convex_hull_order.h"
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>
#include <vector>
namespace cetsp {
class BranchingStrategy {
public:
  virtual void setup(Instance *instance, std::shared_ptr<Node> &root,
                     SolutionPool *solution_pool) {}
  virtual bool branch(Node &node) = 0;
  virtual ~BranchingStrategy() = default;
};

class SequenceRule {
public:
  virtual void setup(const Instance *instance, std::shared_ptr<Node> &root,
                     SolutionPool *solution_pool) = 0;
  virtual bool is_ok(const std::vector<int> &seq) = 0;
  virtual ~SequenceRule() = default;
};

class FarthestCircle : public BranchingStrategy {
  /**
   * This strategy tries to branch on the circle that is most distanced
   * to the relaxed solution.
   */
public:
  explicit FarthestCircle(bool simplify = false) : simplify{simplify} {
    if (simplify) {
      std::cout << "Using node simplification." << std::endl;
    }
  }

  virtual void setup(Instance *instance_, std::shared_ptr<Node> &root,
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

  virtual bool allows_lazy_constraints() { return true; }

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
  Instance *instance = nullptr;
  bool simplify;
  std::vector<std::unique_ptr<SequenceRule>> rules;
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
  explicit ChFarthestCircle(bool simplify = true);
};

class ConvexHullRule : public SequenceRule {
public:
  virtual void setup(const Instance *instance_, std::shared_ptr<Node> &root,
                     SolutionPool *solution_pool);

  static bool
  is_path_sequence_possible(const std::vector<int> &sequence, unsigned int n,
                            const std::vector<bool> &is_in_ch,
                            const std::vector<double> &order_values);
  virtual bool is_ok(const std::vector<int> &seq);

private:
  const Instance *instance = nullptr;
  std::vector<double> order_values;
  std::vector<bool> is_ordered;

  bool sequence_is_ch_ordered(const std::vector<int> &sequence);
  std::vector<Point> get_circle_centers(const Instance &instance) const;
  void compute_weights(const Instance *instance, std::shared_ptr<Node> &root);
};

TEST_CASE("Branching Strategy") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  std::vector<Circle> instance_ = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  Instance instance(instance_);
  FarthestCircle bs;
  auto root = std::make_shared<Node>(std::vector<int>{0, 1, 2, 3}, &instance);
  bs.setup(&instance, root, nullptr);
  CHECK(bs.branch(*root) == false);

  std::vector<Circle> seq = {{{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}};
  Node root2({0, 1, 2}, &instance);
  CHECK(bs.branch(root2) == true);
  CHECK(root2.get_children().size() == 3);
}

TEST_CASE("Path Convex Hull Strategy true") {
  std::vector<Circle> instance_ = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  Instance instance(instance_);
  instance.path = std::optional<std::pair<Point, Point>>({{0, 0}, {1, 1}});

  std::vector<int> sequence = {1, 0, 5, 2, 3, 4};
  unsigned int n = 6;
  std::vector<bool> is_in_ch = {true, true, true, true, true, true};
  std::vector<double> order_values = {0, 1, 2, 3, 4, 5};

  CHECK(ConvexHullRule::is_path_sequence_possible(sequence, n, is_in_ch,
                                                  order_values));
}

TEST_CASE("Path Convex Hull Strategy false") {
  std::vector<Circle> instance_ = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  Instance instance(instance_);
  instance.path = std::optional<std::pair<Point, Point>>({{0, 0}, {1, 1}});

  std::vector<int> sequence = {1, 0, 3, 2, 5, 4};
  unsigned int n = 6;
  std::vector<bool> is_in_ch = {true, true, true, true, true, true};
  std::vector<double> order_values = {0, 1, 2, 3, 4, 5};

  CHECK(!ConvexHullRule::is_path_sequence_possible(sequence, n, is_in_ch,
                                                   order_values));
}

} // namespace cetsp
#endif // CETSP_BRANCHING_STRATEGY_H
