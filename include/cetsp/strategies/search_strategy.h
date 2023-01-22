/**
 * The search strategy defines which node to explore next. Two common
 * strategies are BFS and DFS. DFS will quickly go down (following the best
 * child at every node) to obtain a feasible solution. BFS will be quicker
 * in increasing the lower bound but will take a long time to find a feasible
 * solution. It is also common to switch between DFS and BFS,  i.e., going
 * deep first until a feasible solution is found, then select the cheapest
 * node (usually BFS) to repeat.
 */

#ifndef CETSP_SEARCH_STRATEGY_H
#define CETSP_SEARCH_STRATEGY_H
#include "branching_strategy.h"
#include "cetsp/node.h"

namespace cetsp {

class SearchStrategy {
public:
  virtual void init(std::shared_ptr<Node> &root) = 0;
  /**
   * Gets called after a node has brachned.
   * @param node The node that  just branched.
   */
  virtual void notify_of_branch(Node &node) = 0;
  /**
   * Returns the next node to be explored. This node has to be explored and
   * should not be returned again.
   * @return
   */
  virtual std::shared_ptr<Node> next() = 0;
  /**
   * Checks if there is a node left to explore.
   * @return True if there is a node that requires  exploration. False  if the
   * tree has been fully explored.
   */
  virtual bool has_next() = 0;

  /**
   * Called when the (last) node has been feasible. This allows you for
   * example to switch the strategy every time a new solution has been found.
   * @param node The last node.
   */
  virtual void notify_of_feasible(Node &node){};

  /**
   * Called when the (last) node gets pruned.
   * @param node The last node.
   */
  virtual void notify_of_prune(Node &node){};

  virtual ~SearchStrategy() = default;
};

class DfsBfs : public SearchStrategy {
public:
  void init(std::shared_ptr<Node> &root) override {
    std::cout << "Using DfsBfs search" << std::endl;
    queue.push_back(root);
  }

  void notify_of_branch(Node &node) override {
    auto children = node.get_children();
    std::sort(children.begin(), children.end(),
              [](std::shared_ptr<Node> &a, std::shared_ptr<Node> &b) {
                const auto lb_a = a->get_lower_bound();
                const auto lb_b = b->get_lower_bound();
                if (std::abs(lb_a - lb_b) < 0.001) { // approx equal
                  return a->get_relaxed_solution().obj() >
                         b->get_relaxed_solution().obj();
                }
                return a->get_lower_bound() > b->get_lower_bound();
              });
    for (auto child : children) {
      queue.push_back(child);
    }
  }

  void notify_of_feasible(Node &node) override {
    sort_to_priotize_lowest_value();
  }

  void notify_of_prune(Node &node) override { sort_to_priotize_lowest_value(); }

  std::shared_ptr<Node> next() override {
    if (!has_next()) {
      return nullptr;
    }
    auto n = queue.back();
    queue.pop_back();
    return n;
  }
  bool has_next() override {
    // remove all pruned entries  from  the back
    while (!queue.empty() && queue.back()->is_pruned()) {
      queue.pop_back();
    }
    return !queue.empty();
  }

private:
  void sort_to_priotize_lowest_value() {
    std::sort(queue.begin(), queue.end(),
              [](std::shared_ptr<Node> &a, std::shared_ptr<Node> &b) {
                const auto lb_a = a->get_lower_bound();
                const auto lb_b = b->get_lower_bound();
                if (std::abs(lb_a - lb_b) < 0.001) { // approx equal
                  return a->get_relaxed_solution().obj() >
                         b->get_relaxed_solution().obj();
                }
                return lb_a > lb_b;
              });
  }

  std::vector<std::shared_ptr<Node>> queue;
};
class CheapestChildDepthFirst : public SearchStrategy {
public:
  void init(std::shared_ptr<Node> &root) override { queue.push_back(root); }

  void notify_of_branch(Node &node) override {
    auto children = node.get_children();
    std::sort(children.begin(), children.end(),
              [](std::shared_ptr<Node> &a, std::shared_ptr<Node> &b) {
                const auto lb_a = a->get_lower_bound();
                const auto lb_b = b->get_lower_bound();
                if (std::abs(lb_a - lb_b) < 0.001) { // approx equal
                  return a->get_relaxed_solution().obj() >
                         b->get_relaxed_solution().obj();
                }
                return lb_a > lb_b;
              });
    for (auto &child : children) {
      queue.push_back(child);
    }
  }

  std::shared_ptr<Node> next() override {
    if (!has_next()) {
      return nullptr;
    }
    auto n = queue.back();
    queue.pop_back();
    return n;
  }
  bool has_next() override {
    // remove all pruned entries  from  the back
    while (!queue.empty() && queue.back()->is_pruned()) {
      queue.pop_back();
    }
    return !queue.empty();
  }

private:
  std::vector<std::shared_ptr<Node>> queue;
};
class CheapestBreadthFirst : public SearchStrategy {
public:
  void init(std::shared_ptr<Node> &root) override { queue.push_back(root); }

  void notify_of_branch(Node &node) override {
    for (auto &child : node.get_children()) {
      queue.push_back(child);
    }
    std::sort(queue.begin(), queue.end(),
              [](std::shared_ptr<Node> &a, std::shared_ptr<Node> &b) {
                const auto lb_a = a->get_lower_bound();
                const auto lb_b = b->get_lower_bound();
                if (std::abs(lb_a - lb_b) < 0.001) { // approx equal
                  return a->get_relaxed_solution().obj() >
                         b->get_relaxed_solution().obj();
                }
                return lb_a > lb_b;
              });
  }

  std::shared_ptr<Node> next() override {
    if (!has_next()) {
      return nullptr;
    }
    auto n = queue.back();
    queue.pop_back();
    return n;
  }
  bool has_next() override {
    // remove all pruned entries  from  the back
    while (!queue.empty() && queue.back()->is_pruned()) {
      queue.pop_back();
    }
    return !queue.empty();
  }

private:
  std::vector<std::shared_ptr<Node>> queue;
};

TEST_CASE("Search Strategy") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  Instance instance({{{0, 0}, 1},
                     {{3, 0}, 1},
                     {{6, 0}, 1},
                     {{3, 6}, 1}});
  FarthestCircle bs;
  auto root = std::make_shared<Node>(std::vector<int>{0, 1, 2, 3}, &instance);
  bs.setup(&instance, root, nullptr);
  CheapestChildDepthFirst ss;
  ss.init(root);
  auto node = ss.next();
  CHECK(node != nullptr);
  CHECK(bs.branch(*node) == false);
  ss.notify_of_branch(*node);
  CHECK(ss.next() == nullptr);

  std::vector<Circle> seq = {{{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}};
  auto root2 = std::make_shared<Node>(std::vector<int>{0, 1, 2}, &instance);
  CheapestChildDepthFirst ss2;
  ss2.init(root2);
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
