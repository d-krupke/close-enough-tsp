//
// Created by Dominik Krupke on 14.12.22.
// The task of this file is to fine the root node to start the BnB-algorithm
// with. For a tour, this can be three circles, as the order for three does
// not matter.
//

#ifndef CETSP_ROOT_NODE_STRATEGY_H
#define CETSP_ROOT_NODE_STRATEGY_H
#include "./common.h"
#include "./node.h"
#include "doctest/doctest.h"
#include <vector>
namespace cetsp {
class RootNodeStrategy {
private:
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

public:
  Node get_root_node(const std::vector<Circle> &instance, bool path) {
    if (path) {
      assert(false); // not yet implemented
    }
    if (instance.size() <= 3) { // trivial case
      return Node(instance, &instance);
    }
    auto max_pair = find_max_pair(instance);
    const auto c1 = instance[max_pair.first];
    const auto c2 = instance[max_pair.second];
    auto c3 = instance[max_pair.first];
    double max_dist = 0;
    for (const auto &c : instance) {
      auto dist = c1.center.dist(c.center) + c2.center.dist(c.center);
      if (dist > max_dist) {
        max_dist = dist;
        c3 = c;
      }
    }
    return Node({c1, c2, c3}, &instance);
  }
};

TEST_CASE("Root Node Selection") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  std::vector<Circle> instance = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  RootNodeStrategy rns;
  Node root = rns.get_root_node(instance, false);
  CHECK(root.is_feasible());
}
} // namespace cetsp
#endif // CETSP_ROOT_NODE_STRATEGY_H
