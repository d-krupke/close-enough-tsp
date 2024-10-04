#include <gtest/gtest.h>
#include "cetsp_solver/solution_pool.hpp"

TEST(SolutionPoolTest, KeepsBestTourTest) {
    /**
     * Test that the solution pool keeps the best tour found so far.
     * 
     */
    using namespace cetsp_solver;
    SolutionPool solution_pool;
    
    solution_pool.add_solution(Solution({{0,0}, {1,0}, {1,1}, {0,1}}));
    EXPECT_EQ(solution_pool.get_best_cost(), 4.0);
    solution_pool.add_solution(Solution({{0,0}, {1,0}, {10,1}, {0,1}}));
    EXPECT_EQ(solution_pool.get_best_cost(), 4.0);
    solution_pool.add_solution(Solution({{0,0}, {1,0}}));
    EXPECT_EQ(solution_pool.get_best_cost(), 2.0);
}