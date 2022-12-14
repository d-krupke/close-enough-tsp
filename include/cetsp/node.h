//
// Created by Dominik Krupke on 12.12.22.
//

#ifndef CETSP_NODE_H
#define CETSP_NODE_H
#include "cetsp/common.h"
#include "cetsp/soc.h"
#include "doctest/doctest.h"
#include <numeric>
namespace cetsp {

class Node;
class Node {
public:

  explicit Node(std::vector<Circle> branch_sequence, const Instance *instance,
                Node *parent = nullptr)
      : branch_sequence{std::move(branch_sequence)}, parent{parent},
        instance{instance} {}

  void add_lower_bound(double lb) {
    if (!lower_bound || *lower_bound < lb) {
      lower_bound = lb;
      if (parent != nullptr) {
        parent->reevaluate_children();
      }
    }
  }


  auto get_lower_bound() -> double {
    if (!lower_bound) {
      lower_bound = get_relaxed_solution().length();
    }
    return *lower_bound;
  }



  bool is_feasible() {
    return get_relaxed_solution().covers(instance->begin(), instance->end());
  }

  void branch(std::vector<Node> &&children_) {
    if(is_pruned()) {
      throw std::invalid_argument("Cannot branch on pruned node.");
    }
    assert(!is_feasible());
    if (children_.empty()) {
      prune();
    } else {
      children = std::move(children_);
      reevaluate_children();
    }
  }

  std::vector<Node> &get_children() { return *children; }

  auto get_relaxed_solution() -> const Trajectory&  {
    if (!relaxed_solution) {
      relaxed_solution =compute_tour(branch_sequence);
    }
    return *relaxed_solution;
  }
  void prune() {
    if (pruned) {
      return;
    }
    pruned = true;
    if (children) {
      for (auto child : *children) {
        child.prune();
      }
    }
  }

  const std::vector<Circle>& get_fixed_sequence() {
    return branch_sequence;
  }



  [[nodiscard]] auto is_pruned() const -> bool { return pruned; }

private:
  void reevaluate_children() {
    if (children && !children->empty()) {
      auto lb = std::transform_reduce(
          children->begin(), children->end(), get_lower_bound(),
          [](double a, double b) { return std::min(a, b); },
          [](Node &node) { return node.get_lower_bound(); });
      add_lower_bound(lb);
    }
  }

  std::vector<Circle> branch_sequence;        // fixed part of the solution
  std::optional<Trajectory> relaxed_solution; // relaxed solution
  std::optional<double> lower_bound;
  std::optional<std::vector<Node>> children;
  Node *parent;
  bool pruned = false;
  const Instance *instance;
};

TEST_CASE("Node") {
  std::vector<Circle> seq;
  seq.push_back({{0, 0}, 1});
  seq.push_back({{3, 0}, 1});
  Node node(seq, &seq);
  const auto tour=node.get_relaxed_solution();
  CHECK(tour.length() == doctest::Approx(2.0));
  CHECK(node.get_lower_bound() == doctest::Approx(2.0));
}

} // namespace cetsp
#endif // CETSP_NODE_H
