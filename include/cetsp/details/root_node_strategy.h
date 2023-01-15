//
// Created by Dominik Krupke on 14.12.22.
// The task of this file is to find the root node to start the BnB-algorithm
// with. For a tour, this can be three circles, as the order for three does
// not matter. For a path it is reasonably to start with the circle that is
// most distanced to both end points (we generally want the root node to be
// as expensive as possible and thus close to the cost of the feasible
// solutions)
//

#ifndef CETSP_ROOT_NODE_STRATEGY_H
#define CETSP_ROOT_NODE_STRATEGY_H
#include "cetsp/common.h"
#include "cetsp/node.h"
#include "doctest/doctest.h"
#include <vector>
namespace cetsp {

class RootNodeStrategy {
public:
  virtual std::shared_ptr<Node> get_root_node(Instance &instance) = 0;
  virtual ~RootNodeStrategy() = default;
};

class LongestEdgePlusFurthestCircle : public RootNodeStrategy {

public:
  std::shared_ptr<Node> get_root_node(Instance &instance);
};

class ConvexHull : public RootNodeStrategy {
public:
  std::shared_ptr<Node> get_root_node(Instance &instance);
};

TEST_CASE("Root Node Selection") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  Instance instance;
  instance.push_back({{0, 0}, 1});
  instance.push_back({{3, 0}, 1});
  instance.push_back({{6, 0}, 1});
  instance.push_back({{3, 6}, 1});
  LongestEdgePlusFurthestCircle rns;
  auto root = rns.get_root_node(instance);
  CHECK(root->is_feasible());
}
} // namespace cetsp
#endif // CETSP_ROOT_NODE_STRATEGY_H
