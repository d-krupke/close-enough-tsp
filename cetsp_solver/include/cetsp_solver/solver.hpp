#pragma once
#include "data.hpp"
#include "node.hpp"
#include "search_manager.hpp"
#include "solution_pool.hpp"
#include "worker.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace cetsp_solver {
class Solver {
public:
  Solver(Instance instance_)
      : instance(instance_), solution_pool{}, node_factory{}, search_manager{} {
    solution_pool.set_callback([this](const Solution &solution) {
      std::cout << "New incumbent solution found with cost " << solution.cost
                << ". Lower bound: " << this->get_lower_bound()
                << ", Upper bound: " << this->get_upper_bound() << std::endl;
    });
    search_manager.set_callback(
        [this](const SearchStats &stats) {
          std::cout << "Nodes in frontier: " << stats.num_frontier << std::endl;
        },
        100);
  }

  void solve(double time_limit = 3600) {
    std::cout << "Starting solver" << std::endl;
    auto root_node = node_factory.create_root_node({0, 1, 2});
    search_manager.enqueue_node(std::move(root_node));
    std::vector<std::thread> threads;
    std::vector<std::unique_ptr<SearchWorker>> workers;
    std::condition_variable cv;
    std::mutex cv_m;
    std::atomic<bool> all_workers_done(false);

    for (int i = 0; i < 4; i++) {
      workers.push_back(std::make_unique<SearchWorker>(
          instance, node_factory, search_manager, solution_pool));
      threads.push_back(
          std::thread([this, &workers, i, &all_workers_done, &cv] {
            workers[i]->run();
            if (i == 3) { // Last worker to finish
              all_workers_done.store(true);
              cv.notify_all();
            }
          }));
    }

    // Wait for the workers to finish within the time limit
    std::unique_lock<std::mutex> lk(cv_m);
    if (cv.wait_for(lk, std::chrono::seconds(static_cast<int>(time_limit)),
                    [&all_workers_done] { return all_workers_done.load(); })) {
      // All threads finished within the time limit
      std::cout << "All workers finished within the time limit." << std::endl;
    } else {
      // Time limit exceeded, stop the workers
      std::cout << "Time limit exceeded, stopping workers." << std::endl;
      for (auto &worker : workers) {
        worker->stop();
      }
    }

    // Join all threads
    for (auto &thread : threads) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

  double get_upper_bound() { return solution_pool.get_best_cost(); }

  double get_lower_bound() { return search_manager.get_lower_bound(); }

  std::optional<Solution> get_best_solution() {
    return solution_pool.get_best_solution();
  }

private:
  Instance instance;
  SolutionPool solution_pool;
  NodeFactory node_factory;
  SearchManager search_manager{};
};
} // namespace cetsp_solver