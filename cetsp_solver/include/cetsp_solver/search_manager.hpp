#pragma once
#include "node.hpp"
#include <algorithm>
#include <memory>
#include <mutex>
#include <ranges>
#include <vector>

namespace cetsp_solver {


inline bool min_lb(const Node &a, const Node &b) {
  return a.lb < b.lb;
}

inline bool deepest_min_lb(const Node &a, const Node &b) {
  if (a.depth == b.depth) {
    return min_lb(a, b);
  }
  return a.depth > b.depth;
}

struct AnnotatedNode {
  /**
   * @brief This class stores a node and additional data used by the search
   * manager.
   */

  AnnotatedNode(std::unique_ptr<Node> &&node_) : node(std::move(node_)), id(node->id) {}

  std::unique_ptr<Node> node;
  NodeId id;  // local data with the id of the node. Will never change.
  bool in_process = false;
};

class SearchManager {
  /**
   * @brief This class manages the search process. Its primary function is to
   * provide the next node to be processed by the worker. Workers can also only
   * partially process a node, and return it to the search manager to be
   * completed later.
   *
   */
public:
  SearchManager() {}

  Node* get_next_node(std::function<bool(const Node&, const Node&)> comp) {
      std::lock_guard<std::mutex> lock(mutex);
      if (frontier_nodes.empty()) {
          return nullptr;
      }

      // Find the minimum element among the nodes that are not in process
      auto min_element = std::min_element(frontier_nodes.begin(), frontier_nodes.end(), [&](const AnnotatedNode& a, const AnnotatedNode& b) {
          if (a.in_process) return false;
          if (b.in_process) return true;
          return comp(*a.node, *b.node);
      });

      if (min_element == frontier_nodes.end() || min_element->in_process) {
          return nullptr;
      }

      min_element->in_process = true;
      return min_element->node.get();
  }

  void add_node(std::unique_ptr<Node> &&node) {
    std::lock_guard<std::mutex> lock(mutex);
    frontier_nodes.push_back(AnnotatedNode{std::move(node)});
  }

  void return_node(Node *node) {
    /**
     * @brief Returns a node to the search manager. This means that the node has
     * been partially processed, and it is not finished yet. The worker may only
     * has solved the relaxation of the node, or reinforced the lower bound, but
     * the node is not finished yet.
     */
    if (node == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex);
    std::find_if(frontier_nodes.begin(), frontier_nodes.end(),
                 [node](auto &n) { return n.node.get() == node; })
        ->in_process = false;
  }

  void remove_node(Node *node, bool keep_lower_bound = false) {
    /**
     * @brief Finishes a node. This means that the node has been completely
     * processed, and it is not going to be processed again. The node will be
     * deleted after this method is called.
     */
    std::lock_guard<std::mutex> lock(mutex);
    auto node_id = node->id;
    if (keep_lower_bound) {
      if (node->lb < separate_lb) {
        separate_lb = node->lb;
      }
    }

    frontier_nodes.erase(
        std::remove_if(frontier_nodes.begin(), frontier_nodes.end(),
                       [node_id](const auto &n) {
                         return n.id == node_id;
                       }),
        frontier_nodes.end());
  }

  double get_lower_bound() {
    double lb = separate_lb;
    for (const auto &node : frontier_nodes) {
      lb = std::min(lb, node.node->lb);
    }
    return lb;
  }

  bool is_empty() {
    std::lock_guard<std::mutex> lock(mutex);
    return frontier_nodes.empty();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(mutex);
    return frontier_nodes.size();
  }

private:
  // contains all unfinished nodes. The smallest lower bound within
  // is the best lower bound found so far.
  std::vector<AnnotatedNode> frontier_nodes;
  std::mutex mutex;          // mutex for a thread-safe search management
  double separate_lb = INFINITY; // the worst lower bound of excluded nodes. Will only decrease.
};
} // namespace cetsp_solver