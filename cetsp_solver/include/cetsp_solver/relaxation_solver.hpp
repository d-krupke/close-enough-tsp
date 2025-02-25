#pragma once

#include "data.hpp"
#include "node.hpp"
#include "soc.hpp"
#include <gurobi_c++.h>
#include <vector>

namespace cetsp_solver {
class SocpRelaxationSolver {
  /**
   * @brief This class represents the relaxation of the CE-TSP.
   *
   */
public:
  SocpRelaxationSolver(Instance *instance) : instance(instance) { env.start(); }

  void process_node(Node &node) {
    /**
     * @brief Compute the trajectory for the given sequence of circles in
     * the node and add it as relaxation to the node.
     *
     */
    const auto& circle_sequence = _get_circle_sequence(node);
    auto trajectory = compute_trajectory(circle_sequence, env);
    node.trajectory = trajectory;
    node.add_lower_bound(trajectory.second);
    node.status = NodeStatus::RELAXED;
  }

private:
  std::vector<Circle> _get_circle_sequence(const Node &node) {
    /**
     * @brief Get the circle sequence from the node.
     *
     * @param node The node from which the circle sequence should be extracted.
     * @return std::vector<Circle> The circle sequence.
     */
    const auto &sequence = node.branching_decisions.sequence;
    std::vector<Circle> circle_sequence(sequence.size());
    for (unsigned int i = 0; i < sequence.size(); i++) {
      circle_sequence[i] = (*instance)[sequence[i]];
    }
    return circle_sequence;
  }

  Instance *instance;
  GRBEnv env;
};
} // namespace cetsp_solver