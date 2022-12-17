//
// Created by Dominik Krupke on 17.12.22.
//
#include "cetsp/details/root_node_strategy.h"
namespace cetsp {
std::pair<int, int> find_max_pair(const std::vector<Circle> &instance) {
  double max_dist = 0;
  std::pair<int, int> best_pair;
  for (int i = 0; i < instance.size(); i++) {
    for (int j = 0; j < i; j++) {
      auto dist = instance[i].center.squared_dist(instance[j].center);
      if (dist >= max_dist) {
        best_pair = {i, j};
        max_dist = dist;
      }
    }
  }
  return best_pair;
}

int most_distanced_circle(const Instance &instance) {
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
  if (instance.is_path()) {
    std::vector<int> seq;
    seq.push_back(most_distanced_circle(instance));
    assert(seq[0] < instance.size());
    return Node(seq, &instance);
  } else {
    if (instance.size() <= 3) { // trivial case
      std::vector<int> seq;
      for (int i = 0; i < instance.size(); ++i) {
        seq.push_back(i);
      }
      return Node(seq, &instance);
    }
    auto max_pair = find_max_pair(instance);
    const auto c1 = instance[max_pair.first];
    const auto c2 = instance[max_pair.second];
    int c3 = max_pair.first;
    double max_dist = 0;
    for (int i = 0; i < instance.size(); ++i) {
      const auto &c = instance[i];
      auto dist = c1.center.dist(c.center) + c2.center.dist(c.center);
      if (dist > max_dist) {
        max_dist = dist;
        c3 = i;
      }
    }
    assert(max_pair.first < instance.size());
    assert(max_pair.second < instance.size());
    assert(c3 < instance.size());
    return Node({max_pair.first, c3, max_pair.second}, &instance);
  }
}
} // namespace cetsp