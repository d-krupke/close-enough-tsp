//
// Created by Dominik Krupke on 14.12.22.
//

#ifndef CETSP_BRANCHING_STRATEGY_H
#define CETSP_BRANCHING_STRATEGY_H
#include "common.h"
#include "node.h"
#include <vector>
namespace cetsp {
class BranchingStrategy {
public:
  BranchingStrategy(const Instance *instance) : instance{instance} {}
  bool branch(Node &node) {
    std::vector<double> distances(instance->size());
    for (int i = 0; i < instance->size(); ++i) {
      distances[i] = node.get_relaxed_solution().distance((*instance)[i]);
    }
    auto max_dist = std::max_element(distances.begin(), distances.end());
    if(*max_dist <= 0) {
      return false;
    }
    const int c = std::distance(distances.begin(),max_dist);

    std::vector<Node> children;
    std::vector<int> seqeuence = node.get_fixed_sequence();
    seqeuence.push_back(c);
    if(instance->is_path()) {
      // for path, this position may not be symmetric.
      children.emplace_back(seqeuence, instance, &node);
    }
    for(int i = seqeuence.size()-1; i>0; --i) {
      seqeuence[i] = seqeuence[i-1];
      seqeuence[i-1] = c;
      children.emplace_back(seqeuence, instance, &node);
    }
    node.branch(std::move(children));
    return true;
  }

private:
  const Instance *instance;
};
TEST_CASE("Branching Strategy") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  std::vector<Circle> instance_ = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  Instance instance(instance_);
  BranchingStrategy bs(&instance);
  Node root({0,1,2,3}, &instance);
  CHECK(bs.branch(root) == false);

  std::vector<Circle> seq = {
    {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}};
  Node root2({0,1,2}, &instance);
  CHECK(bs.branch(root2) == true);
  CHECK(root2.get_children().size()==3);
}
}
#endif // CETSP_BRANCHING_STRATEGY_H
