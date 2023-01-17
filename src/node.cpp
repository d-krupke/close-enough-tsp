//
// Created by Dominik Krupke on 12.12.22.
//

#include "cetsp/node.h"
namespace cetsp {
void Node::add_lower_bound(const double lb) {
  if (get_lower_bound() < lb) {
    lazy_lower_bound_value = lb;
    // propagate to parent
    if (parent != nullptr && parent->get_lower_bound() < lb) {
      parent->reevaluate_children();
    }
    // Potentially also propagate to children.
    if (!children.empty()) {
      for (auto &child : children) {
        child->add_lower_bound(lb);
      }
    }
  }
}

auto Node::get_lower_bound() -> double {
  if (!lazy_lower_bound_value) {
    lazy_lower_bound_value = get_relaxed_solution().obj();
    if (parent != nullptr) {
      if (lazy_lower_bound_value < parent->get_lower_bound()) {
        lazy_lower_bound_value = parent->get_lower_bound();
      }
    }
  }
  return *lazy_lower_bound_value;
}

bool Node::is_feasible() { return _relaxed_solution.is_feasible(); }

void Node::branch(std::vector<std::shared_ptr<Node>> &children_) {
  if (is_pruned()) {
    throw std::invalid_argument("Cannot branch on pruned node.");
  }
  assert(!is_feasible());
  if (children_.empty()) {
    prune();
    children = std::vector<std::shared_ptr<Node>>{};
  } else {
    children = children_;
    reevaluate_children();
  }
}

auto Node::get_relaxed_solution() -> const PartialSequenceSolution & {
  return _relaxed_solution;
}

void Node::prune(bool infeasible) {
  if (pruned) {
    return;
  }
  pruned = true;
  if (infeasible) {
    add_lower_bound(std::numeric_limits<double>::infinity());
  }
  for (auto &child : children) {
    child->prune(infeasible);
  }
}

void Node::reevaluate_children() {
  if (!children.empty()) {
    auto lb = std::transform_reduce(
        children.begin(), children.end(),
        std::numeric_limits<double>::infinity(),
        [](double a, double b) { return std::min(a, b); },
        [](std::shared_ptr<Node> &node) { return node->get_lower_bound(); });
    // std::cout << "Reevaluated children at depth "<<depth()<< " to
    // "<<lb<<std::endl;
    add_lower_bound(lb);
  }
}
} // namespace cetsp
