/**
 * @brief In this file, we implement an algorithm that lower bounds the
 * necessary costs to integrate the missing disks into the current solution.
 * This can be used to improve the current problem relaxation, which can be used
 * to prune the search tree earlier. It should be especially useful for
 * instances with small radii. It should theoretically converge to a similar
 * relaxation as the classical TSP for radii approaching zero, but it is more
 * expensive because we need to consider the order of the already fixed
 * solution. Without involving the given order, this approach would only be
 * strong at the root node, but we need it to be strong deeper in the search
 * tree, where we already made some `wrong' decisions.
 *
 * @author Dominik Krupke
 * @version 0.1
 * @date 2023-07-20
 *
 */
#ifndef CETSP_MISSING_DISKS_LB_H
#define CETSP_MISSING_DISKS_LB_H
#include "cetsp/bnb.h"
#include "cetsp/callbacks.h"
#include "cetsp/common.h"
#include "doctest/doctest.h"
#include <cmath>
#include <gurobi_c++.h>
#include <nlopt.hpp>
#include <type_traits>
#include <unordered_map>

namespace cetsp {

namespace {
std::vector<int> compute_dispersed_set_of_missing_disks(
    const Instance &instance, std::vector<int> &fixed_tour, int max_size) {
  /**
   * @brief Compute a set of missing disks that are dispersed. This is used to
   * compute a lower bound on the necessary costs to integrate the missing disks
   * into the current solution. The runtime should be subquadratic, thus
   * negligible.
   *
   * @param instance The instance.
   * @return std::vector<int> The set of missing disks.
   */

  // This distance can become negative on purpose!
  auto dist = [](Circle a, Circle b) {
    return a.center.dist(b.center) - (a.radius + b.radius);
  };

  std::vector<double> distances;
  // fill with infinity
  distances.resize(instance.size(), std::numeric_limits<double>::infinity());
  for (int c : fixed_tour) {
    for (int i = 0; i < instance.size(); ++i) {
      distances[i] = std::min(distances[i], dist(instance[c], instance[i]));
    }
  }
  int n_remaining = max_size - fixed_tour.size();
  std::vector<int> result;
  for (int i = 0; i < n_remaining; ++i) {
    auto it = std::max_element(distances.begin(), distances.end());
    if (*it <= 0) {
      break; // Discs that intersect other discs can harm the solution.
    }
    auto idx = std::distance(distances.begin(), it);
    result.push_back(idx);
    for (int i = 0; i < instance.size(); ++i) {
      distances[i] = std::min(distances[i], dist(instance[idx], instance[i]));
    }
  }
  return result;
}

// A hashable triple
using Triple = std::tuple<int, int, int>;
struct TripleHash {
  std::size_t operator()(const Triple &k) const {
    auto [u, v, w] = k;
    return ((std::hash<int>()(u) ^ (std::hash<int>()(v) << 1)) >> 1) ^
           (std::hash<int>()(w) << 1);
  }
};

// Some C++ magic to convert lambdas into functions for nlopt

template <class L, class R, class... Args> static auto impl_impl(L l) {
  static_assert(!std::is_same<L, std::function<R(Args...)>>::value,
                "Only lambdas are supported, it is unsafe to use "
                "std::function or other non-lambda callables");

  static L lambda_s = std::move(l);
  return +[](Args... args) -> R { return lambda_s(args...); };
}

template <class L>
struct to_f_impl : public to_f_impl<decltype(&L::operator())> {};

template <class ClassType, class R, class... Args>
struct to_f_impl<R (ClassType::*)(Args...) const> {
  template <class L> static auto impl(L l) {
    return impl_impl<L, R, Args...>(std::move(l));
  }
};

template <class ClassType, class R, class... Args>
struct to_f_impl<R (ClassType::*)(Args...)> {
  template <class L> static auto impl(L l) {
    return impl_impl<L, R, Args...>(std::move(l));
  }
};

template <class L> auto to_f(L l) { return to_f_impl<L>::impl(std::move(l)); }

class InsertionCostCalculator {
  /**
   * @brief This class is used to calculate the cost of inserting a disk
   * between two others. This is used to calculate a lower bound on the
   * necessary costs to integrate the missing disks into the current solution.
   *
   * Because the computation may be expensive, this class caches the results
   * and, thus, you should not create multiple instances of this class for
   * the same problem instance but share it.
   */
public:
  InsertionCostCalculator(Instance *instance) : instance{instance} {
    if (instance->is_path()) {
      throw std::runtime_error(
          "InsertionCostCalculator is only implemented for tours.");
    }
  }

  double calculate_lb_on_insertion_costs(int u, int v, int w) {
    /**
     * @brief Calculate the cost of inserting the disk v between disks u and w.
     * This cost should be symmetric, i.e., the order of u and w should not
     * matter. Because the computation may be expensive, this is cached in a
     * map. The actual computation is done in `compute_cost`.
     */
    if (w < u) { // making triple unique independ of direction
      std::swap(u, w);
    }
    Triple uvw(u, v, w);
    if (map.count(uvw) > 0) {
      return map[uvw];
    }
    auto l = compute_cost((*instance)[u], (*instance)[v], (*instance)[w]);
    map[uvw] = l;
    return l;
  }

protected:
  virtual double compute_cost(Circle u, Circle v, Circle w) {
    /**
     * @brief Compute the cost of inserting the disk v between disks u and w.
     * This cost should be symmetric, i.e., the order of u and w should not
     * matter.
     */
    /*auto t = compute_tour({u, v, w}, true);
    return (t.points[0].dist(t.points[1]) + t.points[1].dist(t.points[2])) -
           t.points[0].dist(t.points[2]);*/

    auto l = compute_tour({u, v, w}, true).length();
    l -= u.center.dist(w.center) + u.radius + w.radius;
    return std::max(0.0, l);
  }

  Instance *instance;
  std::unordered_map<Triple, double, TripleHash> map;
};

class ExactInsertionCostCalculator : public InsertionCostCalculator {
  /**
   * @brief This class is used to calculate the exact inserting cost
   * of a disk between two others.
   */
public:
  [[maybe_unused]] explicit ExactInsertionCostCalculator(Instance *instance)
      : InsertionCostCalculator(instance) {}

protected:
  double compute_cost(Circle u, Circle v, Circle w) override {
    /**
     * @brief Compute the minimum cost of inserting the disk v between disks u
     * and w. This cost should be symmetric, i.e., the order of u and w should
     * not matter.
     */

    auto objFunction = [&u, &v, &w](unsigned n, const double *x, double *grad,
                                    void *my_func_data) {
      if (grad) {
        // We use derivative-free optimization algorithms.
        throw std::runtime_error(
            "We only use derivative-free optimization algorithms!");
      }

      double angle0 = x[0];
      double angle1 = x[1];
      double angle2 = x[2];

      auto a = Point(u.center.x + std::cos(angle0) * u.radius,
                     u.center.y + std::sin(angle0) * u.radius);
      auto b = Point(v.center.x + std::cos(angle1) * v.radius,
                     v.center.y + std::sin(angle1) * v.radius);
      auto c = Point(w.center.x + std::cos(angle2) * w.radius,
                     w.center.y + std::sin(angle2) * w.radius);

      return a.dist(b) + b.dist(c) - a.dist(c);
    };

    try {
      nlopt::opt opt(nlopt::GN_ISRES, 3);
      opt.set_lower_bounds({0, 0, 0});
      opt.set_upper_bounds({2 * M_PI, 2 * M_PI, 2 * M_PI});
      opt.set_xtol_rel(1e-6);
      opt.set_ftol_rel(1e-6);

      opt.set_min_objective(to_f(objFunction), nullptr);
      opt.set_maxtime(0.01); // 10 ms

      std::vector<double> x{0, 0, 0};
      double minimum;

      auto result = opt.optimize(x, minimum);

      if (result == NLOPT_MAXTIME_REACHED) {
        return InsertionCostCalculator::compute_cost(u, v, w);
      }

      return minimum;
    } catch (std::exception &e) {
      std::cout << "nlopt failed: " << e.what() << std::endl;
      throw std::runtime_error(e.what());
    }
  }

  Instance *instance;
  std::unordered_map<Triple, double, TripleHash> map;
};

using Tuple = std::tuple<int, int>;
struct TupleHash {
  std::size_t operator()(const cetsp::Tuple &k) const {
    auto [u, v] = k;
    return ((std::hash<int>()(u) ^ (std::hash<int>()(v) << 1)) >> 1);
  }
};

template <class CostCalculator> class SegmentIntegrationVars {
  /**
   * @brief This is the LP-part for a single segment.
   *
   */
public:
  SegmentIntegrationVars(GRBModel &model, CostCalculator &cost_calculator,
                         std::pair<int, int> segment,
                         std::vector<int> &missing_disks)
      : model{model}, cost_calculator{cost_calculator}, segment{segment},
        missing_disks{missing_disks} {
    create_variables();
    enforce_single_path();
    enforce_flow_conservation();
    prohibit_small_loops();
  }

  GRBLinExpr get_var_inserted_into_segment(int c) {
    /**
     * @brief Returns a linear expression that evaluates to 1 if the disk c is
     * inserted into the segment.
     */
    return get_incoming(c);
  }

  GRBLinExpr get_cost() {
    /**
     * @brief Returns the cost of the integration into this segment.
     *
     */
    GRBLinExpr cost = 0;
    for (const auto &[t, var] : x) {
      auto [a, b] = t;
      auto cost_ab =
          cost_calculator.calculate_lb_on_insertion_costs(a, b, segment.second);
      cost += cost_ab * var;
    }
    return cost;
  }

private:
  void create_variables() {
    // create variables
    for (int a : missing_disks) {
      x[{segment.first, a}] =
          model.addVar(/*lb=*/0, /*ub=*/1, /*obj=*/0, /*type=*/GRB_CONTINUOUS);
    }
    for (int a : missing_disks) {
      for (int b : missing_disks) {
        if (a == b) {
          continue;
        }
        x[{a, b}] = model.addVar(/*lb=*/0, /*ub=*/1, /*obj=*/0,
                                 /*type=*/GRB_CONTINUOUS);
      }
    }
  }

  void prohibit_small_loops() {
    for (int a : missing_disks) {
      for (int b : missing_disks) {
        if (a == b) {
          continue;
        }
        model.addConstr(x[{a, b}] + x[{b, a}] <= 1);
      }
    }
  }

  GRBLinExpr get_incoming(int c) {
    GRBLinExpr incoming = x[{segment.first, c}];
    for (int a : missing_disks) {
      if (a == c) {
        continue;
      }
      incoming += x[{a, c}];
    }
    return incoming;
  }

  GRBLinExpr get_outgoing(int c) {
    GRBLinExpr outgoing = 0;
    for (int b : missing_disks) {
      if (b == c) {
        continue;
      }
      outgoing += x[{c, b}];
    }
    return outgoing;
  }

  void enforce_single_path() {
    model.addConstr(get_outgoing(segment.first) <= 1);
  }

  void enforce_flow_conservation() {
    for (int c : missing_disks) {
      GRBLinExpr incoming = get_incoming(c);
      GRBLinExpr outgoing = get_outgoing(c);
      model.addConstr(outgoing <= incoming);
    }
  }

  GRBModel &model;
  CostCalculator &cost_calculator;
  std::pair<int, int> segment;
  std::vector<int> &missing_disks;
  std::unordered_map<Tuple, GRBVar, TupleHash> x;
};

// TODO: Combined model.

template <class CostCalculator> class CircleIntegrationCostModel {
public:
  CircleIntegrationCostModel(GRBEnv &env, Instance &instance,
                             std::vector<int> &fixed_tour,
                             std::vector<int> &missing_disks,
                             CostCalculator &cost_calculator)
      : model{&env}, instance{instance}, fixed_tour{fixed_tour},
        missing_disks{missing_disks}, cost_calculator{cost_calculator} {
    GRBLinExpr obj = 0;
    for (int i = 0; i < fixed_tour.size(); ++i) {
      submodels.emplace_back(
          model, cost_calculator,
          std::make_pair(fixed_tour[i],
                         fixed_tour[(i + 1) % fixed_tour.size()]),
          missing_disks);
      obj += submodels.back().get_cost();
    }
    model.setObjective(obj, GRB_MINIMIZE);

    for (auto c : missing_disks) {
      GRBLinExpr covered = 0;
      for (auto &submodel : submodels) {
        covered += submodel.get_var_inserted_into_segment(c);
      }
      model.addConstr(covered == 1);
    }
  }

  double compute_lb() {
    model.set(GRB_IntParam_OutputFlag, 0);
    model.optimize();
    return model.get(GRB_DoubleAttr_ObjBound);
  }

private:
  GRBModel model;
  std::vector<SegmentIntegrationVars<CostCalculator>> submodels;
  Instance &instance;
  std::vector<int> &fixed_tour;
  std::vector<int> &missing_disks;
  CostCalculator &cost_calculator;
};

} // namespace

template <class CostCalculator>
class LowerBoundImprovingCallback : public B2BNodeCallback {
public:
  LowerBoundImprovingCallback(Instance &instance)
      : instance{instance}, cost_calculator{&instance} {
    std::cout << "Created LB callback" << std::endl;
  }

  void on_entering_node(EventContext &context) override {
    if (context.current_node->get_lower_bound() >
        1.01 * context.get_relaxed_solution().obj()) {
      return; // We already have a good lb
    }

    double gap = 1 - context.get_lower_bound() / context.get_upper_bound();
    if (false && context.current_node->depth() != 1 && gap > 0.1) {
      return;
    }
    /* if (context.get_lower_bound() <=
     0.99*context.current_node->get_lower_bound()) { return;
     }*/
    if (context.is_feasible()) {
      return;
    }
    auto fixed_tour = context.current_node->get_fixed_sequence();
    auto missing_disks =
        compute_dispersed_set_of_missing_disks(instance, fixed_tour, 50);
    if (missing_disks.empty()) {
      std::cout << "No missing disks" << std::endl;
      return;
    }
    static GRBEnv env;
    std::cout << "Building LB model" << std::endl;
    CircleIntegrationCostModel model(env, instance, fixed_tour, missing_disks,
                                     cost_calculator);
    std::cout << "Solving LB model" << std::endl;
    auto lb = model.compute_lb();
    std::cout << "improving lb by " << lb << " working on "
              << missing_disks.size() << " disks" << std::endl;
    auto obj = context.current_node->get_relaxed_solution().obj();
    std::cout << "old lb: " << obj << std::endl;
    std::cout << "new lb " << obj + lb << std::endl;
    context.current_node->add_lower_bound(obj + lb);
    std::cout << "Check lb " << context.current_node->get_lower_bound()
              << std::endl;
  }

private:
  Instance &instance;
  CostCalculator cost_calculator;
};

// TODO: Implement a callback that adds the lb to the node

} // namespace cetsp

TEST_CASE("Dispersion") {
  cetsp::Instance instance;
  instance.push_back({{0, 0}, 1});
  instance.push_back({{10, 0}, 1});
  instance.push_back({{10, 10}, 1});
  instance.push_back({{0, 10}, 1});
  instance.push_back({{5, 5}, 1});
  instance.push_back({{1, 0}, 1});
  std::vector seq = {0, 1, 2, 3};
  auto res = cetsp::compute_dispersed_set_of_missing_disks(instance, seq, 10);
  CHECK(res.size() == 1);
}

TEST_CASE("Lower Bound") {
  cetsp::Instance instance;
  instance.push_back({{0, 0}, 0.1});
  instance.push_back({{10, 0}, 0.1});
  instance.push_back({{10, 10}, 0.1});
  instance.push_back({{0, 10}, 0.1});
  instance.push_back({{5, 5}, 0.1});
  instance.push_back({{2.5, 2.5}, 0.1});
  instance.push_back({{7.5, 5}, 0.1});
  instance.push_back({{1, 0}, 0.1});
  std::vector seq = {0, 1, 2, 3};
  auto res = cetsp::compute_dispersed_set_of_missing_disks(instance, seq, 10);
  GRBEnv env;
  cetsp::InsertionCostCalculator cc(&instance);
  cetsp::CircleIntegrationCostModel model(env, instance, seq, res, cc);
  CHECK(model.compute_lb() == doctest::Approx(3.954));
}

TEST_CASE("LB Callback") {
  using namespace cetsp;
  Instance instance;
  for (double x = 0; x <= 10; x += 2.0) {
    for (double y = 0; y <= 10; y += 2.0) {
      instance.push_back({{x, y}, 1});
    }
  }
  LongestEdgePlusFurthestCircle root_node_strategy{};
  FarthestCircle branching_strategy;
  CheapestChildDepthFirst search_strategy;
  BranchAndBoundAlgorithm bnb(&instance,
                              root_node_strategy.get_root_node(instance),
                              branching_strategy, search_strategy);
  auto callback = std::make_unique<
      cetsp::LowerBoundImprovingCallback<InsertionCostCalculator>>(instance);
  bnb.add_node_callback(std::move(callback));
  bnb.optimize(30);
  CHECK(bnb.get_solution());
  CHECK(bnb.get_upper_bound() <= 41);
}

#endif // CETSP_MISSING_DISKS_LB_H