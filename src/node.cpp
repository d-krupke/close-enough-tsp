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
      for(auto&  child: children) {
        child.add_lower_bound(lb);
      }
    }
  }
}

auto Node::get_lower_bound() -> double {
  if (!lazy_lower_bound_value) {
    lazy_lower_bound_value = get_relaxed_solution().length();
    if (parent != nullptr) {
      if (lazy_lower_bound_value < parent->get_lower_bound()) {
        lazy_lower_bound_value = parent->get_lower_bound();
      }
    }
  }
  return *lazy_lower_bound_value;
}

bool Node::is_feasible() {
  if (instance->revision == feasible_revision) {
    return true;
  }
  if (feasible_revision == -2) {
    return false;
  }
  if (get_relaxed_solution().covers(instance->begin(), instance->end(),
                                    instance->eps)) {
    feasible_revision = instance->revision;
    return true;
  } else {
    feasible_revision = -2;
    return false;
  }
}

void Node::branch(std::vector<Node> &&children_) {
  if (is_pruned()) {
    throw std::invalid_argument("Cannot branch on pruned node.");
  }
  assert(!is_feasible());
  if (children_.empty()) {
    prune();
    children = std::vector<Node>{};
  } else {
    children = std::move(children_);
    reevaluate_children();
  }
}

auto Node::get_relaxed_solution() -> const Trajectory & {
  if (!relaxed_solution) {
    if (instance->is_tour()) {
      std::vector<Circle> circles;
      circles.reserve(branch_sequence.size());
      for (auto i : branch_sequence) {
        assert(i < static_cast<int>(instance->size()));
        circles.push_back((*instance).at(i));
      }
      assert(circles.size() == branch_sequence.size());
      auto soc = compute_tour_with_spanning_information(circles, false);
      relaxed_solution = std::move(soc.first);
      spanning_circles = std::move(soc.second);
    } else {
      std::vector<Circle> circles;
      circles.reserve(branch_sequence.size() + 2);
      circles.push_back(Circle(instance->path->first, 0));
      for (auto i : branch_sequence) {
        circles.push_back((*instance).at(i));
      }
      circles.push_back(Circle(instance->path->second, 0));
      assert(circles.size() == branch_sequence.size() + 2);
      std::tie(relaxed_solution, spanning_circles) = compute_tour_with_spanning_information(circles, true);
    }
  }
  return *relaxed_solution;
}

void Node::prune() {
  if (pruned) {
    return;
  }
  pruned = true;
  add_lower_bound(std::numeric_limits<double>::infinity());
  for (auto &child : children) {
    child.prune();
  }
}

void Node::reevaluate_children() {
  if (!children.empty()) {
    auto lb = std::transform_reduce(
        children.begin(), children.end(),
        std::numeric_limits<double>::infinity(),
        [](double a, double b) { return std::min(a, b); },
        [](Node &node) {
          return (node.is_pruned() ? std::numeric_limits<double>::infinity()
                                   : node.get_lower_bound());
        });
    // std::cout << "Reevaluated children at depth "<<depth()<< " to
    // "<<lb<<std::endl;
    add_lower_bound(lb);
  }
}
} // namespace cetsp
