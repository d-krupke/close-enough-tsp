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
  explicit Node(std::vector<int> branch_sequence, const Instance *instance,
                Node *parent = nullptr)
      : branch_sequence{std::move(branch_sequence)}, parent{parent},
        instance{instance} {
    for(const auto& i: branch_sequence) {
      assert(i<instance->size());
    }
  }

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

  bool is_feasible(double eps) {
    return get_relaxed_solution().covers(instance->begin(), instance->end(), eps);
  }

  void branch(std::vector<Node> &&children_) {
    if (is_pruned()) {
      throw std::invalid_argument("Cannot branch on pruned node.");
    }
    assert(!is_feasible(0.0));
    if (children_.empty()) {
      prune();
      children = std::vector<Node>{};
    } else {
      children = std::move(children_);
      reevaluate_children();
    }
  }

  const std::vector<Node> &get_children() const {
    return children;
  }
  std::vector<Node> &get_children() {
    return children;
  }

  auto get_relaxed_solution() -> const Trajectory & {
    if (!relaxed_solution) {
      if(instance->is_tour()) {
        std::vector<Circle> circles;
        circles.reserve(branch_sequence.size());
        for(auto i: branch_sequence){
          assert(i<instance->size());
          circles.push_back((*instance).at(i));
        }
        assert(circles.size() == branch_sequence.size());
        relaxed_solution = compute_tour(circles, false);
      } else {
        std::vector<Circle> circles;
        circles.reserve(branch_sequence.size()+2);
        circles.push_back(Circle(instance->path->first, 0));
        for(auto i: branch_sequence){
          circles.push_back((*instance).at(i));
        }
        circles.push_back(Circle(instance->path->second, 0));
        assert(circles.size() == branch_sequence.size()+2);
        relaxed_solution = compute_tour(circles, true);
      }
    }
    return *relaxed_solution;
  }
  void prune() {
    if (pruned) {
      return;
    }
    pruned = true;
    add_lower_bound(std::numeric_limits<double>::infinity());
    for (auto child : children) {
      child.prune();
    }
  }

  const std::vector<int> &get_fixed_sequence() { return branch_sequence; }

  [[nodiscard]] auto is_pruned() const -> bool { return pruned; }

private:
  void reevaluate_children() {
    if (!children.empty()) {
      auto lb = std::transform_reduce(
          children.begin(), children.end(), std::numeric_limits<double>::infinity(),
          [](double a, double b) { return std::min(a, b); },
          [](Node &node) {
            return (node.is_pruned() ? std::numeric_limits<double>::infinity()
                                     : node.get_lower_bound());
          });
      add_lower_bound(lb);
    }
  }

  std::vector<int> branch_sequence;        // fixed part of the solution
  std::optional<Trajectory> relaxed_solution; // relaxed solution
  std::optional<double> lower_bound;
  std::vector<Node> children;
  Node *parent;
  bool pruned = false;
  const Instance *instance;
};

TEST_CASE("Node") {
  Instance seq;
  seq.push_back({{0, 0}, 1});
  seq.push_back({{3, 0}, 1});
  CHECK(seq.is_tour());
  Node node({0,1}, &seq);
  const auto tour = node.get_relaxed_solution();
  CHECK(tour.length() == doctest::Approx(2.0));
  CHECK(node.get_lower_bound() == doctest::Approx(2.0));
  CHECK(node.is_feasible(0.01));
}

} // namespace cetsp
#endif // CETSP_NODE_H
