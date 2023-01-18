//
// Created by Dominik Krupke on 12.12.22.
//

#include "cetsp/node.h"
namespace cetsp {

static bool is_segments_intersect(const Point &p11, const Point &p12,
                                  const Point &p21, const Point &p22);

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
      std::tie(relaxed_solution, spanning_circles) =
          compute_tour_with_spanning_information(circles, true);
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

std::vector<TrajectoryIntersection> Node::get_intersections() {
  const auto &trajectory = get_relaxed_solution();
  /* currently only paths are supported */
  assert(trajectory.points.front() == trajectory.points.back());
  /* We assume each disc has a point in the trajectory */
  assert(trajectory.points.size() - 1 == instance->size());

  /* Collect all edges */
  std::vector<
      std::tuple<const Point &, const Point &, const Circle &, const Circle &>>
      edges;
  for (unsigned int i = 0; i < instance->size(); i++) {
    unsigned int j = (i + 1) % instance->size();
    const Point &p1 = trajectory.points[i];
    const Point &p2 = trajectory.points[j];
    const Circle &c1 = (*instance).at(i);
    const Circle &c2 = (*instance).at(j);
    edges.push_back(std::make_tuple(p1, p2, c1, c2));
  }

  /* Search for intersections */
  std::vector<TrajectoryIntersection> intersections;
  for (unsigned int i = 0; i < edges.size(); i++) {
    for (unsigned int j = 0; j < edges.size(); j++) {
      unsigned int i_prev = ((int)i - 1) % edges.size();
      unsigned int i_next = (i + 1) % edges.size();
      if (j == i_prev || j == i || j == i_next)
        continue;
      auto const &a = edges[i];
      auto const &b = edges[j];
      if (is_segments_intersect(std::get<0>(a), std::get<1>(a), std::get<0>(b),
                                std::get<1>(b))) {
        intersections.push_back(TrajectoryIntersection(
            std::get<0>(a), std::get<1>(a), std::get<2>(a), std::get<3>(a),
            std::get<0>(b), std::get<1>(b), std::get<2>(b), std::get<3>(b)));
      }
    }
  }
  return intersections;
}

static bool is_segments_intersect(const Point &p11, const Point &p12,
                                  const Point &p21, const Point &p22) {
  auto ccw = [](const Point &a, const Point &b, const Point &c) {
    return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
  };

  return ccw(p11, p21, p22) != ccw(p12, p21, p22) &&
         ccw(p11, p12, p21) != ccw(p11, p12, p22);
}

} // namespace cetsp
