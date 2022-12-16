//
// Created by Dominik Krupke on 14.12.22.
// This file provide logic of keeping track of the solutions found and
// the corresponding upper bounds.
//

#ifndef CETSP_SOLUTION_POOL_H
#define CETSP_SOLUTION_POOL_H
#include "common.h"
namespace cetsp {
class SolutionPool {
public:
  void add_solution(const Trajectory &solution) {
    if (solution.length() < ub) {

      solutions.push_back(solution);
      ub = solution.length();
    }
  }
  double get_upper_bound() { return ub; }

  std::unique_ptr<Trajectory> get_best_solution() {
    if (solutions.empty()) {
      return nullptr;
    }
    return std::make_unique<Trajectory>(solutions.back());  // best solution is always at the end
  }

  bool empty() { return solutions.empty(); }

private:
  double ub = std::numeric_limits<double>::infinity();
  std::vector<Trajectory> solutions;
};
} // namespace cetsp
#endif // CETSP_SOLUTION_POOL_H
