#include <gtest/gtest.h>
#include "cetsp_solver/data.hpp"
#include "cetsp_solver/soc.hpp"

TEST(SECOND_ORDER_CONE_PROGRAM, SimpleTest) {
    using namespace cetsp_solver;
    std::vector<Circle> circle_sequence = {Circle(Point(0, 0), 1), Circle(Point(10, 0), 1)};
    GRBEnv env;
    auto [trajectory, obj] = compute_trajectory(circle_sequence, env);
    EXPECT_EQ(trajectory.size(), circle_sequence.size());
    EXPECT_NEAR(trajectory[0].x, 1.0, 0.001);
    EXPECT_NEAR(trajectory[0].y, 0.0, 0.001);
    EXPECT_NEAR(trajectory[1].x, 9.0, 0.001);
    EXPECT_NEAR(trajectory[1].y, 0.0, 0.001);
}