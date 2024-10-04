#pragma once
#include "data.hpp"
#include "search_manager.hpp"
#include "solution_pool.hpp"
#include "node.hpp"
#include "worker.hpp"

namespace cetsp_solver
{
class Solver
{
public:
    Solver(Instance instance_): instance(instance_), solution_pool{}, node_factory{}, search_manager{} 
     {}

    void solve() {
        auto root_node = node_factory.create_root_node({0,1,2});
        search_manager.add_node(std::move(root_node));
        std::vector<std::thread> workers;
        for (int i = 0; i < 4; i++) {
            workers.push_back(std::thread([this] {
                SearchWorker worker(instance, node_factory, search_manager, solution_pool);
                worker.run();
            }));
        }
        for (auto &worker : workers) {
            worker.join();
        }
    }

    double get_upper_bound() {
        return solution_pool.get_best_cost();
    }

    double get_lower_bound() {
        return search_manager.get_lower_bound();
    }

    std::optional<Solution> get_best_solution() {
        return solution_pool.get_best_solution();
    }

    Instance instance;
    SolutionPool solution_pool;
    NodeFactory node_factory;
    SearchManager search_manager{};

};
} // namespace cetsp_solver
