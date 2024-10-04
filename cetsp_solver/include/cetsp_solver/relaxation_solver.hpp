#pragma once

#include "data.hpp"
#include <gurobi_c++.h>
#include <vector>
#include "node.hpp"
#include "soc.hpp"

namespace cetsp_solver {
    class SocpRelaxationSolver {
  /**
   * @brief This class represents the relaxation of the CE-TSP.
   *
   */
public:
  SocpRelaxationSolver(Instance *instance) : instance(instance) {
    env.start();
  }

  void process_node(Node &node) {
    /**
     * @brief Compute the trajectory for the given sequence of circles in
     * the node and add it as relaxation to the node.
     *
     */
    std::vector<Circle> circle_sequence(node.sequence.size());
    for (unsigned int i = 0; i < node.sequence.size(); i++) {
      circle_sequence[i] = (*instance)[node.sequence[i]];
    }
    auto trajectory = compute_trajectory(circle_sequence, env);
    node.trajectory = trajectory;
    node.add_lower_bound(trajectory.second);
    node.status = NodeStatus::RELAXED;
  }

private:
  Instance *instance;
  GRBEnv env;
};
} // namespace cetsp_solver