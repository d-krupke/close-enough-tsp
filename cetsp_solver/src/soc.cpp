#include "cetsp_solver/soc.hpp"
#include <assert.h>
#include <numeric>

namespace cetsp_solver {
std::pair<std::vector<Point>, double>
compute_trajectory(const std::vector<Circle> &circle_sequence, GRBEnv &env) {
  GRBModel model(env);

  const auto n = circle_sequence.size();
  std::vector<GRBVar> x(n);
  std::vector<GRBVar> y(n);
  std::vector<GRBVar> f(n);
  std::vector<GRBVar> w(n);
  std::vector<GRBVar> u(n);
  std::vector<GRBVar> s(n);
  std::vector<GRBVar> t(n);
  

  for (unsigned i = 0; i < n; ++i) {
    x[i] = model.addVar(/*lb=*/-GRB_INFINITY, /*ub=*/GRB_INFINITY,
                        /*obj=*/0.0, /*vtype=*/GRB_CONTINUOUS);
    y[i] = model.addVar(/*lb=*/-GRB_INFINITY, /*ub=*/GRB_INFINITY,
                        /*obj=*/0.0, /*vtype=*/GRB_CONTINUOUS
                        /*name=*/);
    f[i] = model.addVar(/*lb=*/0, /*ub=*/GRB_INFINITY,
                        /*obj=*/0.0, /*vtype=*/GRB_CONTINUOUS
                        /*name=*/);
    w[i] = model.addVar(/*lb=*/-GRB_INFINITY, /*ub=*/GRB_INFINITY,
                        /*obj=*/0.0, /*vtype=*/GRB_CONTINUOUS
                        /*name=*/);
    u[i] = model.addVar(/*lb=*/-GRB_INFINITY, /*ub=*/GRB_INFINITY,
                        /*obj=*/0.0, /*vtype=*/GRB_CONTINUOUS
                        /*name=*/);
    s[i] = model.addVar(/*lb=*/-GRB_INFINITY, /*ub=*/GRB_INFINITY,
                        /*obj=*/0.0, /*vtype=*/GRB_CONTINUOUS
                        /*name=*/);
    t[i] = model.addVar(/*lb=*/-GRB_INFINITY, /*ub=*/GRB_INFINITY,
                        /*obj=*/0.0, /*vtype=*/GRB_CONTINUOUS
                        /*name=*/);
  }
  GRBLinExpr obj = std::accumulate(f.begin(), f.end(), GRBLinExpr(0.0));

  model.setObjective(obj, GRB_MINIMIZE);

  for (unsigned i = 0; i < n; ++i) {
    model.addQConstr(f[i] * f[i] >= w[i] * w[i] + u[i] * u[i]);
    const auto r = circle_sequence[i].radius;
    model.addQConstr(s[i] * s[i] + t[i] * t[i] <= r * r);

    const auto cx = circle_sequence[i].center.x;
    const auto cy = circle_sequence[i].center.y;
    model.addConstr(s[i] == cx - x[i]);
    model.addConstr(t[i] == cy - y[i]);
  }

  for (unsigned i = 0; i < n; ++i) {
    const auto prev_c = (i == 0 ? n - 1 : i - 1);
    assert(prev_c >= 0);
    model.addConstr(w[i] == x[prev_c] - x[i]);
    model.addConstr(u[i] == y[prev_c] - y[i]);
  }
  model.set(GRB_IntParam_OutputFlag, 0);
  // tuned via the built-in tune() function of Gurobi.
  model.set(GRB_IntParam_Presolve, 0);
  model.set(GRB_IntParam_SimplexPricing, 3);
  // model.set(GRB_IntParam_PrePasses, 8);
  model.optimize();
  std::vector<Point> points;
  points.reserve(n + 1);
  for (unsigned i = 0; i < n; i++) {
    points.emplace_back(x[i].get(GRB_DoubleAttr_X), y[i].get(GRB_DoubleAttr_X));
  }
  return {points, model.get(GRB_DoubleAttr_ObjVal)};
}
} // namespace cetsp_solver