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
    } else {
      std::cout << "discard solution worse  than  ub" << std::endl;
    }
  }
  double get_upper_bound() { return ub; }

  const Trajectory &get_best_solution() {
    if (solutions.empty()) {
      throw std::exception();
    }
    return solutions.back();
  }

  bool empty() { return solutions.empty(); }

private:
  double ub = std::numeric_limits<double>::infinity();
  std::vector<Trajectory> solutions;
};
} // namespace cetsp
#endif // CETSP_SOLUTION_POOL_H
