#include <gtest/gtest.h>
#include "cetsp_solver/solver.hpp"

TEST(SolverTest, SimpleTest) {
    using namespace cetsp_solver;
    Instance instance{{{0,0}, 0}, {{0,0.5},0}, {{1,0}, 0}, {{1.0,0.5}, 0}, {{1,1}, 0}, {{0,1}, 0}};
    Solver solver(instance);
    solver.solve();
    auto upper_bound = solver.get_upper_bound();
    auto lower_bound = solver.get_lower_bound();
    EXPECT_NEAR(upper_bound, lower_bound, 0.001);
    EXPECT_NEAR(upper_bound, 4.0, 0.001);
}

TEST(SolverTest, LargerTest) {
    using namespace cetsp_solver;
    std::vector<Circle> circles;
    for (float x= 0.0; x < 6.0; x += 1.0) {
        for (float y = 0.0; y < 6.0; y += 1.0) {
            circles.push_back(Circle(Point(x, y), 0.5));
        }
    }
    Instance instance{circles};
    Solver solver(instance);
    solver.solve();
    auto upper_bound = solver.get_upper_bound();
    auto lower_bound = solver.get_lower_bound();
    EXPECT_NEAR(upper_bound, lower_bound, 0.001);
    EXPECT_NEAR(upper_bound, 4.7070953689185426, 0.001);
}
