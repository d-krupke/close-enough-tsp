//
// Created by Dominik Krupke on 19.01.23.
//

#ifndef CETSP_CONVEX_HULL_RULE_H
#define CETSP_CONVEX_HULL_RULE_H
#include "cetsp/common.h"
#include "cetsp/details/solution_pool.h"
#include "cetsp/node.h"
#include "cetsp/strategies/rule.h"
namespace cetsp {

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
#endif // CETSP_CONVEX_HULL_RULE_H
