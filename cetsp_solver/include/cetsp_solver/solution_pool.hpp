#pragma once
#include "data.hpp"
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>


namespace cetsp_solver {
class SolutionPool {
  /**
   * @brief This class manages the solutions found by the workers. It is used to
   * store the best solution found so far, and to store the solutions that are
   * found by the workers.
   */
public:
  using Callback = std::function<void(const Solution &)>;

  void add_solution(const Solution &solution) {
    /**
     * @brief Adds a solution to the pool. If the solution is better than the
     * best solution found so far, it will be stored as the best solution.
     *
     * @param solution The solution to be added.
     * @param cost The cost of the solution.
     */
    {
      std::lock_guard<std::mutex> lock(mutex_incumbent);
      if (!best_solution.has_value() || solution.cost < best_solution->cost) {
        best_solution = solution;
      }
      solutions.push_back(solution);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_callback);
      if (incumbent_update_callback) {
        incumbent_update_callback(solution);
      }
    }
  }

  std::optional<Solution> get_best_solution() {
    /**
     * @brief Returns the best solution found so far.
     *
     * @return Trajectory The best solution found so far.
     */
    std::lock_guard<std::mutex> lock(mutex_incumbent);
    return best_solution;
  }

  double get_best_cost() {
    /**
     * @brief Returns the cost of the best solution found so far.
     *
     * @return double The cost of the best solution found so far.
     */
    std::lock_guard<std::mutex> lock(mutex_incumbent);
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
    std::lock_guard<std::mutex> lock(mutex_incumbent);
    return solutions;
  }

  void clear() {
    /**
     * @brief Clears the pool. This means that all the solutions will be
     * deleted.
     */
    std::lock_guard<std::mutex> lock(mutex_incumbent);
    solutions.clear();
    best_solution.reset();
  }

  void set_incumbent_update_callback(Callback callback) {
    /**
     * @brief Sets the callback function to be called whenever the incumbent
     * solution is updated.
     *
     * @param callback The callback function.
     */
    std::lock_guard<std::mutex> lock(mutex_incumbent);
    incumbent_update_callback = callback;
  }

private:
  std::mutex mutex_incumbent;
  std::mutex mutex_callback;
  std::optional<Solution> best_solution;
  std::vector<Solution> solutions;
  Callback incumbent_update_callback;
};
} // namespace cetsp_solver