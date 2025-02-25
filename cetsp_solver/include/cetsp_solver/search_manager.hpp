// This file defines a search manager that will manage the search process. It
// is responsible for providing the next node to be processed by the workers,
// and to keep track of the nodes that are currently in the frontier.
// It is essentially a more dynamic version of a priority queue, which can
// be used to build a search tree in a branch-and-bound algorithm.

#pragma once
#include "node.hpp"
#include <algorithm>
#include <memory>
#include <mutex>
#include <ranges>
#include <vector>

namespace cetsp_solver {

inline bool min_lb(const Node &a, const Node &b) { return a.lb < b.lb; }

inline bool deepest_min_lb(const Node &a, const Node &b) {
  if (a.depth == b.depth) {
    return min_lb(a, b);
  }
  return a.depth > b.depth;
}

struct NodeInfo {
  /**
   * @brief This struct stores basic information about a node. It is used by the
   * search stats to store information about the last node processed. This is
   * primarily used for logging purposes.
   */
  NodeId id;
  double lb;
  int depth;
  NodeStatus status;
};

inline NodeInfo get_info(Node *node) {
  /**
   * @brief Returns the basic information about a node.
   *
   * @param node The node for which the information should be returned.
   * @return NodeInfo The information about the node.
   */
  return NodeInfo{node->id, node->lb, node->depth, node->status};
}

struct NodeHandle {
  /**
   * @brief This class stores a node and additional data used by the search
   * manager.
   */

  NodeHandle(std::unique_ptr<Node> &&node_)
      : node(std::move(node_)), id(node->id) {}

  std::unique_ptr<Node> node;
  NodeId id; // local data with the id of the node. Will never change.
  bool in_process =
      false; // used to indicate if the node is currently processed by a worker
};

struct SearchStats {
  /**
   * @brief This struct stores statistics about the search process for logging
   * purposes.
   *
   */
  size_t num_nodes = 0;
  size_t num_frontier = 0;
  size_t num_closed_nodes = 0;
  size_t num_nodes_requeued = 0;
  size_t num_nodes_requested = 0;
  std::optional<NodeInfo> last_node = std::nullopt;
};

struct SearchManagerCallbacks {
  using NextNodeCallback = std::function<void(Node *, const SearchStats &)>;
  using CloseNodeCallback = std::function<void(Node&, bool, const SearchStats &)>;
  using RequeueNodeCallback = std::function<void(Node&, const SearchStats &)>;
  using EnqueueNodeCallback = std::function<void(Node&,const SearchStats &)>;

  NextNodeCallback next_node = nullptr;
  CloseNodeCallback close_node = nullptr;
  RequeueNodeCallback requeue_node = nullptr;
  EnqueueNodeCallback enqueue_node = nullptr;

  void call_cb_next_node(Node *node, const SearchStats &stats) {
    if (next_node) {
      next_node(node, stats);
    }
  }

  void call_cb_close_node(Node &node, bool keep_lb, const SearchStats &stats) {
    if (close_node) {
      close_node(node, keep_lb, stats);
    }
  }

  void call_cb_requeue_node(Node &node, const SearchStats &stats) {
    if (requeue_node) {
      requeue_node(node, stats);
    }
  }

  void call_cb_enqueue_node(Node &node, const SearchStats &stats) {
    if (enqueue_node) {
      enqueue_node(node, stats);
    }
  }
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

  Node *get_next_node(std::function<bool(const Node &, const Node &)> comp) {
    /**
     * @brief Returns the next node to be processed by the workers. The node is
     * selected based on the comparison function comp. The function should
     * return true if the first argument is better than the second argument.
     *
     * @param comp The comparison function used to select the next node.
     * @return Node* The next node to be processed.
     */
    std::lock_guard<std::mutex> lock(mutex);
    if (frontier_nodes.empty()) {
      callbacks.call_cb_next_node(nullptr, stats);
      return nullptr;
    }

    // Find the minimum element among the nodes that are not in process
    auto min_element =
        std::min_element(frontier_nodes.begin(), frontier_nodes.end(),
                         [&](const NodeHandle &a, const NodeHandle &b) {
                           if (a.in_process)
                             return false;
                           if (b.in_process)
                             return true;
                           return comp(*a.node, *b.node);
                         });

    if (min_element == frontier_nodes.end() || min_element->in_process) {
      callbacks.call_cb_next_node(nullptr, stats);
      return nullptr;
    }
    stats.num_nodes_requested++;
    min_element->in_process = true;
    auto* node = min_element->node.get();
    callbacks.call_cb_next_node(node, stats);
    return node;
  }

  void enqueue_node(std::unique_ptr<Node> &&node) {
    /**
     * @brief Adds a node to the search manager. The node will be processed by
     * the workers.
     *
     */
    std::lock_guard<std::mutex> lock(mutex);
    callbacks.call_cb_enqueue_node(*node, stats);
    stats.num_nodes++;
    frontier_nodes.push_back(NodeHandle{std::move(node)});
  }

  void requeue_node(Node *node) {
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
    auto it = std::find_if(frontier_nodes.begin(), frontier_nodes.end(),
                           [node](auto &n) { return n.node.get() == node; });
    it->in_process = false;
    callbacks.call_cb_requeue_node(*node, stats);
    stats.last_node = get_info(node);
    stats.num_nodes_requeued++;
  }

  void close_node(Node *node, bool keep_lower_bound = false) {
    /**
     * @brief Finishes a node. This means that the node has been completely
     * processed, and it is not going to be processed again. The node will be
     * deleted after this method is called.
     */
    callbacks.call_cb_close_node(*node, keep_lower_bound, stats);
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto node_id = node->id;
      if (keep_lower_bound) {
        if (node->lb < separate_lb) {
          separate_lb = node->lb;
        }
      }
  
      frontier_nodes.erase(
          std::remove_if(frontier_nodes.begin(), frontier_nodes.end(),
                         [node_id](const auto &n) { return n.id == node_id; }),
          frontier_nodes.end());

      // Update the last node processed
      stats.last_node = get_info(node);
      stats.num_closed_nodes++;
    }
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

  SearchStats get_stats() {
    std::lock_guard<std::mutex> lock(mutex);
    stats.num_frontier = frontier_nodes.size();
    return stats;
  }



SearchManagerCallbacks callbacks;

private:
  // contains all unfinished nodes. The smallest lower bound within
  // is the best lower bound found so far.
  std::vector<NodeHandle> frontier_nodes;
  std::mutex mutex;  // mutex for a thread-safe search management
  SearchStats stats; // statistics about the search process

  double separate_lb =
      INFINITY; // the worst lower bound of excluded nodes. Will only decrease.
};
} // namespace cetsp_solver