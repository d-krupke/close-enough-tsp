// The branching strategy decides how to branch a node. The branching strategy
// is responsible for creating child nodes based on the current node. The
// branching strategy is used by the search worker to branch a node.

#pragma once

#include "node.hpp"
#include "relaxation.hpp"

namespace cetsp_solver {

class BranchingStrategy {
public:
  BranchingStrategy(NodeFactory &node_factory, SearchManager &search_manager)
      : node_factory(node_factory), search_manager(search_manager) {}

  bool branch_node(Node &node) {
    auto [idx, dist] = node.annotated_trajectory->get_max_distance();
    // place the circle with the index at every possible position in the
    // sequence
    for (uint64_t i = 1; i <= node.sequence.size(); i++) {
      auto new_sequence = node.sequence;
      new_sequence.insert(new_sequence.begin() + i, idx);
      create_child(node, new_sequence);
    }
    finalize_parent(node);
    return true;
  }

private:
  std::vector<int> get_sequence(const Node &node) {
    return node.sequence;
  }

  void create_child(Node &node, std::vector<int> new_sequence) {
    auto new_node = node_factory.create_child_node(node, new_sequence);
    search_manager.enqueue_node(std::move(new_node));
  }

  void finalize_parent(Node &node) {
    node.status = NodeStatus::BRANCHED;
    search_manager.close_node(&node, /*keep_lower_bound=*/false);
  }

  NodeFactory &node_factory;
  SearchManager &search_manager;
};
} // namespace cetsp_solver