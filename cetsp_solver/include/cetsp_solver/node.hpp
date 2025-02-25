#pragma once
#include "data.hpp"
#include "relaxation.hpp"
#include <mutex>
#include <optional>
#include <vector>

namespace cetsp_solver {
enum class NodeStatus {
  /**
   * @brief A node can have different statuses during the branch-and-bound
   * process. Instead of enforcing each node to be processed in one step,
   * we allow the worker to partially process a node, and return it to the
   * search manager to be completed later. This enum represents the different
   * stages of a node.
   */
  UNKNOWN,             // Node has been created but not yet processed
  PREPROCESSED,        // The node has been preprocessed
  RELAXED,             // The relaxation of the node has been solved
  PRUNED,              // The node has been pruned
  INCOMPLETE,          // The solution of the node is incomplete and needs to be branched
  BRANCHED,            // The node has been branched/expanded
  FEASIBLE,            // The node has been proven to be feasible
};

struct BranchingDecisions {
  /**
   * @brief This struct stores the branching decisions made by the worker.
   * It is used to store the branching decisions made by the worker, and to
   * store the branching decisions that are made by the worker.
   */
  std::vector<int> sequence;
};

using NodeId = int;
class Node {
  /**
   * @brief This class represents a node in the branch-and-bound tree.
   * Add all the data necessary for the processing of the node, including
   * intermediate results between the different stages of the node processing.
   *
   */
public:
  Node(int id = -1, NodeId parent_id = -1, int depth = 0, double initial_lb = 0.0,
    BranchingDecisions branching_decisions = {})
      : id(id), parent_id(parent_id), depth(depth), lb(initial_lb),
      branching_decisions(branching_decisions) {}
  NodeId id;
  NodeId parent_id;
  int depth;
  double lb;
  BranchingDecisions branching_decisions;
  std::optional<RelaxedSolution> trajectory;
  std::optional<AnnotatedRelaxedSolution> annotated_trajectory;
  NodeStatus status = NodeStatus::UNKNOWN;

  bool add_lower_bound(double new_lb) {
    if (new_lb > lb) {
      lb = new_lb;
      return true;
    }
    return false;
  }
};

class NodeFactory {
  /**
   * @brief Create nodes for the branch-and-bound tree. This class is
   * thread-safe and ensures that each node has a unique id. It will also
   * automatically assign some values based on the parent node.
   */
public:
  std::unique_ptr<Node> create_root_node(BranchingDecisions branching_decisions) {
    std::lock_guard<std::mutex> lock(mutex);
    return std::make_unique<Node>(next_id++, -1, 0, 0.0, branching_decisions);
  }
  std::unique_ptr<Node> create_child_node(const Node &parent,
                                          const BranchingDecisions &branching_decisions) {
    std::lock_guard<std::mutex> lock(mutex);
    return std::make_unique<Node>(next_id++, parent.id, parent.depth + 1,
                                  parent.lb, branching_decisions);
  }

private:
  NodeId next_id = 0;
  std::mutex mutex;
};
} // namespace cetsp_solver
