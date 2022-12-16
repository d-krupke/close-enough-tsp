//
// Created by Dominik Krupke on 15.12.22.
//

#ifndef CETSP_SEARCH_STRATEGY_H
#define CETSP_SEARCH_STRATEGY_H
#include "cetsp/details/branching_strategy.h"
#include "cetsp/node.h"

namespace cetsp {
class SearchStrategy {
public:
  SearchStrategy(Node &root) { queue.push_back(&root); }
  void notify_of_branch(Node &node) {
    std::vector<Node*> children;
    for (auto &child : node.get_children()) {
      children.push_back(&child);
    }
    std::sort(children.begin(), children.end(), [](Node* a, Node*  b) {
      return a->get_lower_bound() > b->get_lower_bound();
    });
    for(auto child: children){
      queue.push_back(child);
    }
  }

  Node *next() {
    if (!has_next()) {
      return nullptr;
    }
    Node *n = queue.back();
    queue.pop_back();
    return n;
  }
  bool has_next() {
    // remove all pruned entries  from  the back
    while (!queue.empty() && queue.back()->is_pruned()) {
      queue.pop_back();
    }
    return !queue.empty();
  }

private:
  std::vector<Node *> queue;
};

TEST_CASE("Search Strategy") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  Instance instance ({
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}});
  BranchingStrategy bs(&instance);
  Node root({0,1,2,3}, &instance);
  SearchStrategy ss(root);
  auto* node = ss.next();
  CHECK(node!= nullptr);
  CHECK(bs.branch(*node) == false);
  ss.notify_of_branch(*node);
  CHECK(ss.next() == nullptr);

  std::vector<Circle> seq = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}};
  Node root2({0,1,2}, &instance);
  SearchStrategy ss2(root2);
  node = ss2.next();
  CHECK(bs.branch(*node) == true);
  ss2.notify_of_branch(*node);
  CHECK(ss2.next() != nullptr);
  CHECK(ss2.next() != nullptr);
  CHECK(ss2.next() != nullptr);
  CHECK(ss2.next() == nullptr);
}

} // namespace cetsp
#endif // CETSP_SEARCH_STRATEGY_H
