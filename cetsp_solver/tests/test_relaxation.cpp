#include <gtest/gtest.h>
#include "cetsp_solver/relaxation_solver.hpp"

TEST(RelaxationTest, SimpleTest) {
    using namespace cetsp_solver;
    Instance instance = {Circle(Point(0, 0), 1), Circle(Point(10, 0), 1)};
    std::vector<int> sequence = {0, 1};
    NodeFactory nf;
    auto node = nf.create_root_node(sequence);
    EXPECT_EQ(bool(node->trajectory), false);
    SocpRelaxationSolver solver(&instance);
    solver.process_node(*node);
    EXPECT_EQ(bool(node->trajectory), true);
    EXPECT_NEAR(node->lb, 16.0, 0.001);
}