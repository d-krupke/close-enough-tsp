//
// Created by Dominik Krupke on 17.12.22.
//
#include "cetsp/details/root_node_strategy.h"
#include "cetsp/details/cgal_kernel.h"

#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>
namespace cetsp {
std::pair<int, int> find_max_pair(const std::vector<Circle> &instance) {
  /**
   * Find the circle pair with the longest distance  between its centers.
   */
  double max_dist = 0;
  std::pair<int, int> best_pair;
  for (unsigned i = 0; i < instance.size(); i++) {
    for (unsigned j = 0; j < i; j++) {
      auto dist = instance[i].center.squared_dist(instance[j].center);
      if (dist >= max_dist) {
        best_pair = {i, j};
        max_dist = dist;
      }
    }
  }
  return best_pair;
}

auto most_distanced_circle(const Instance &instance) {
  assert(instance.path);
  auto p0 = instance.path->first;
  auto p1 = instance.path->second;
  auto max_el = std::max_element(
      instance.begin(), instance.end(), [&](const Circle &a, const Circle &b) {
        return p0.dist(a.center) + p1.dist(a.center) <
               p0.dist(b.center) + p1.dist(b.center);
      });
  return std::distance(instance.begin(), max_el);
}
Node LongestEdgePlusFurthestCircle::get_root_node(Instance &instance) {
  /**
   * Compute a  root note consisting of three circles by first finding
   * the most distanced pair and then adding a third circle that has the
   * longest sum of  distance to the two end points.
   */
  if (instance.is_path()) {
    std::vector<int> seq;
    seq.push_back(static_cast<int>(most_distanced_circle(instance)));
    assert(seq[0] < static_cast<int>(instance.size()));
    return Node(seq, &instance);
  } else {
    if (instance.size() <= 3) { // trivial case
      std::vector<int> seq;
      for (int i = 0; i < static_cast<int>(instance.size()); ++i) {
        seq.push_back(i);
      }
      return Node(seq, &instance);
    }
    auto max_pair = find_max_pair(instance);
    const auto c1 = instance[max_pair.first];
    const auto c2 = instance[max_pair.second];
    auto c3 = max_pair.first;
    double max_dist = 0;
    for (unsigned i = 0; i < instance.size(); ++i) {
      const auto &c = instance[i];
      auto dist = c1.center.dist(c.center) + c2.center.dist(c.center);
      if (dist > max_dist) {
        max_dist = dist;
        c3 = i;
      }
    }
    assert(max_pair.first < static_cast<int>(instance.size()));
    assert(max_pair.second < static_cast<int>(instance.size()));
    assert(c3 < static_cast<int>(instance.size()));
    return Node({max_pair.first, c3, max_pair.second}, &instance);
  }
}

Node ConvexHull::get_root_node(Instance &instance) {
  if (instance.is_path()) {
    throw std::invalid_argument("ConvexHull Strategy only feasible for tours.");
  }
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point_2;
  typedef CGAL::Convex_hull_traits_adapter_2<
      K, CGAL::Pointer_property_map<Point_2>::type>
      Convex_hull_traits_2;
  std::vector<Point_2> points;
  points.reserve(instance.size());
  for (const auto &c : instance) {
    points.emplace_back(c.center.x, c.center.y);
  }
  std::vector<int> indices(points.size()), out;
  std::iota(indices.begin(), indices.end(), 0);
  CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                      Convex_hull_traits_2(CGAL::make_property_map(points)));
  std::vector<Circle> ch_circles;
  for (auto i : out) {
    ch_circles.push_back(instance[i]);
  }
  //  Only use circles that are  explicitly contained.
  const auto traj =
      compute_tour_with_spanning_information(ch_circles, /*path=*/false);
  std::vector<int> sequence;
  std::copy_if(out.begin(), out.end(), std::back_inserter(sequence),
               [&traj](auto i) { return traj.second[i]; });
  return Node{out, &instance};
}
} // namespace cetsp
