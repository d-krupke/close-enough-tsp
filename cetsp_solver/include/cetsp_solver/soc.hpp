#pragma once
#include "./data.hpp"
#include <gurobi_c++.h>
#include <vector>

namespace cetsp_solver {
std::pair<std::vector<Point>, double>
compute_trajectory(const std::vector<Circle> &circle_sequence, GRBEnv &env);
} // namespace cetsp_solver