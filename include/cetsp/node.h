/**
 * Implements a node in the BnB tree.
 */
#ifndef CETSP_NODE_H
#define CETSP_NODE_H
#include "cetsp/common.h"
#include "cetsp/soc.h"
#include "doctest/doctest.h"
#include <numeric>
namespace cetsp {

class Node {
public:
  Node(Node &node) = delete;
  Node(Node &&node) = default;
  explicit Node(std::vector<int> branch_sequence, Instance *instance,
                Node *parent = nullptr)
      : branch_sequence{std::move(branch_sequence)}, parent{parent},
        instance{instance} {
    for (const auto &i : branch_sequence) {
      assert(i < instance->size());
    }
  }

  void add_lower_bound(double lb);

  auto get_lower_bound() -> double;

  bool is_feasible();

  void branch(std::vector<Node> &&children_);

  const std::vector<Node> &get_children() const { return children; }
  std::vector<Node> &get_children() { return children; }

  auto get_relaxed_solution() -> const Trajectory &;
  void prune();

  [[nodiscard]] const std::vector<int> &get_fixed_sequence() {
    return branch_sequence;
  }

  [[nodiscard]] auto is_pruned() const -> bool { return pruned; }

  [[nodiscard]] Instance *get_instance() { return instance; }

private:
  void reevaluate_children();

  std::vector<int> branch_sequence;           // fixed part of the solution
  std::optional<Trajectory> relaxed_solution; // relaxed solution
  std::optional<double> lower_bound;
  std::vector<Node> children;
  Node *parent;
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
  CHECK(tour.length() == doctest::Approx(2.0));
  CHECK(node.get_lower_bound() == doctest::Approx(2.0));
  CHECK(node.is_feasible());
}

} // namespace cetsp
#endif // CETSP_NODE_H
