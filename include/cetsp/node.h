/**
 * Implements a node in the BnB tree.
 */
#ifndef CETSP_NODE_H
#define CETSP_NODE_H
#include "cetsp/common.h"
#include "cetsp/soc.h"
#include "doctest/doctest.h"
#include "relaxed_solution.h"
#include <numeric>
namespace cetsp {

class Node {
public:
  Node(Node &node) = delete;
  Node(Node &&node) = default;
  explicit Node(std::vector<int> branch_sequence_, Instance *instance,
                Node *parent = nullptr)
      : _relaxed_solution(instance, std::move(branch_sequence_)),
        parent{parent}, instance{instance} {
    if (parent != nullptr) {
      _depth = parent->depth() + 1;
    }
  }

  void add_lower_bound(double lb);

  auto get_lower_bound() -> double;

  bool is_feasible();

  void branch(std::vector<Node> &&children_);

  const std::vector<Node> &get_children() const { return children; }
  [[nodiscard]] std::vector<Node> &get_children() { return children; }

  [[nodiscard]] Node *get_parent() { return parent; }
  [[nodiscard]] const Node *get_parent() const { return parent; }

  auto get_relaxed_solution() -> const PartialSequenceSolution &;

  /**
   * Will prune the node, i.e., mark it as not leading to an optimal solution
   * and thus stopping at it. Pruned nodes are allowed to be deleted from
   * memory.
   */
  void prune();

  [[nodiscard]] const std::vector<int> &get_fixed_sequence() {
    return _relaxed_solution.get_sequence();
  }

  /**
   * The spanning sequence is a subset of the fixed sequence, but with
   * all indices belonging to circles that to not span/define the trajectroy
   * removed.
   * @return The orded list of indices of the circles spanning the current
   * trajectory.
   */
  [[nodiscard]] std::vector<int> get_spanning_sequence() {
    std::vector<int> spanning_sequence;
    int n = static_cast<int>(_relaxed_solution.get_sequence().size());
    spanning_sequence.reserve(n);
    for (int i = 0; i < n; ++i) {
      if (_relaxed_solution.is_sequence_index_spanning(i)) {
        spanning_sequence.push_back(_relaxed_solution.get_sequence()[i]);
      }
    }
    return spanning_sequence;
  }

  [[nodiscard]] auto is_pruned() const -> bool { return pruned; }

  [[nodiscard]] Instance *get_instance() { return instance; }

  [[nodiscard]] int depth() const { return _depth; }

private:
  // Check if the children allow to improve the lower bound.
  void reevaluate_children();

  PartialSequenceSolution _relaxed_solution;
  std::optional<double> lazy_lower_bound_value;
  std::vector<Node> children;
  Node *parent;

  int _depth = 0;
  bool pruned = false;
  Instance *instance;
  int feasible_revision = -1;
};

TEST_CASE("Node") {
  Instance seq;
  seq.push_back({{0, 0}, 1});
  seq.push_back({{3, 0}, 1});
  CHECK(seq.is_tour());
  Node node({0, 1}, &seq);
  const auto tour = node.get_relaxed_solution();
  CHECK(tour.obj() == doctest::Approx(2.0));
  CHECK(node.get_lower_bound() == doctest::Approx(2.0));
  CHECK(node.is_feasible());
}

} // namespace cetsp
#endif // CETSP_NODE_H
