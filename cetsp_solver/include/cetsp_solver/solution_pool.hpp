#pragma once
#include "data.hpp"
#include <mutex>
#include <optional>
#include <vector>
#include <cmath>

namespace cetsp_solver {
class SolutionPool {
  /**
   * @brief This class manages the solutions found by the workers. It is used to
   * store the best solution found so far, and to store the solutions that are
   * found by the workers.
   */
public:
  void add_solution(const Solution &solution) {
    /**
     * @brief Adds a solution to the pool. If the solution is better than the
     * best solution found so far, it will be stored as the best solution.
     *
     * @param solution The solution to be added.
     * @param cost The cost of the solution.
     */
    std::lock_guard<std::mutex> lock(mutex);
    if (!best_solution.has_value() || solution.cost < best_solution->cost) {
      best_solution = solution;
    }
    solutions.push_back(solution);
  }

  std::optional<Solution> get_best_solution() {
    /**
     * @brief Returns the best solution found so far.
     *
     * @return Trajectory The best solution found so far.
     */
    std::lock_guard<std::mutex> lock(mutex);
    return best_solution;
  }

  double get_best_cost() {
    /**
     * @brief Returns the cost of the best solution found so far.
     *
     * @return double The cost of the best solution found so far.
     */
    std::lock_guard<std::mutex> lock(mutex);
    if (best_solution.has_value()) {
      return best_solution->cost;
    }
    return INFINITY;
  }

  std::vector<Solution> get_solutions() {
    /**
     * @brief Returns all the solutions found so far.
     *
     * @return std::vector<Trajectory> All the solutions found so far.
     */
    std::lock_guard<std::mutex> lock(mutex);
    return solutions;
  }

  void clear() {
    /**
     * @brief Clears the pool. This means that all the solutions will be
     * deleted.
     */
    std::lock_guard<std::mutex> lock(mutex);
    solutions.clear();
    best_solution = std::nullopt;
  }

private:
  std::vector<Solution> solutions; // contains all the solutions found so far
  std::optional<Solution> best_solution; // the best solution found so far
  std::mutex mutex;
};
} // namespace cetsp_solver
