//
// Created by Dominik Krupke on 11.12.22.
//

#ifndef CETSP_HEURISTICS_H
#define CETSP_HEURISTICS_H
#include "cetsp/common.h"
#include "doctest/doctest.h"
namespace cetsp {
/**
 * Compute a heuristic solution using a procedure based on 2-Opt.
 * For this, only the circle's centers are considered, which
 * can lead to quite suboptimal solutions in some cases.
 */
auto compute_tour_by_2opt(Instance &instance) -> PartialSequenceSolution;

Trajectory tour_lns(const Instance &instance, const Trajectory &trajectory,
                    int begin, int end) {
  assert(instance.is_tour());
  assert(std::all_of(
      instance.begin(), instance.end(),
      [&trajectory](const auto &c) { return trajectory.covers(c, 0.001); }));
  auto fixed_trajectory = trajectory.sub(end, begin);
  const auto lns_path_begin = fixed_trajectory.points.back();
  const auto lns_path_end = fixed_trajectory.points.front();
  Instance missing_circles;
  for (const auto &c : instance) {
    if (!fixed_trajectory.covers(c, 0.001)) {
      missing_circles.push_back(c);
    }
  }
  missing_circles.path = {lns_path_begin, lns_path_end};
  std::cout << missing_circles.size() << " circles  to be  recovered"
            << std::endl;
  // solve subinstance
  std::unique_ptr<RootNodeStrategy> rns;
  rns = std::make_unique<LongestEdgePlusFurthestCircle>();
  std::unique_ptr<BranchingStrategy> branching_strategy;
  branching_strategy = std::make_unique<ChFarthestCircle>(true);
  std::unique_ptr<SearchStrategy> search_strategy;
  search_strategy = std::make_unique<DfsBfs>();
  BranchAndBoundAlgorithm baba(&missing_circles,
                               rns->get_root_node(missing_circles),
                               *branching_strategy, *search_strategy);
  auto initial_solution = trajectory.sub(begin, end);
  assert(std::all_of(missing_circles.begin(), missing_circles.end(),
                     [&initial_solution](const auto &c) {
                       return initial_solution.covers(c, 0.001);
                     }));
  baba.add_upper_bound(initial_solution);
  baba.optimize(30);
  // build new solution
  auto sol = baba.get_solution();
  if (!sol) {
    return trajectory;
  }
  assert(!sol->is_tour());
  std::vector<Point> points;
  for (const auto &p : fixed_trajectory.points) {
    points.push_back(p);
  }
  for (const auto &p : sol->points) {
    if (points.back() != p) {
      points.push_back(p);
    }
  }
  assert(points.back().squared_dist(points.back()) <= 0.01);
  points.back() = points.front();
  return Trajectory{points};
}

Trajectory optimize_tour_by_lns(const Instance &instance, Trajectory trajectory,
                                int iterations, int k) {
  for (int i = 0; i < iterations; ++i) {
    int n = trajectory.points.size() - 1;
    k = std::min(n - 2, k);
    int begin = rand() % n;
    int end = (begin + k) % n;
    trajectory = tour_lns(instance, trajectory, begin, end);
  }
  return trajectory;
}

TEST_CASE("2Opt") {
  const std::vector<Circle> seq = {
      {{0, 0}, 0}, {{1, 1}, 0}, {{1, 0}, 0}, {{0, 1}, 0}};
  auto instance = Instance(seq);
  auto traj = compute_tour_by_2opt(instance);
  CHECK(traj.obj() == doctest::Approx(4));

  const std::vector<Circle> seq2 = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{5, 2}, 1}, {{3, 3}, 1}, {{0, 4}, 1}};
  auto instance2 = Instance(seq2);
  traj = compute_tour_by_2opt(instance2);
  CHECK(traj.obj() >= 1);
}

TEST_CASE("LNS") {
  const std::vector<Circle> seq2 = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{5, 2}, 1}, {{3, 3}, 1}, {{0, 4}, 1}};
  auto instance = Instance(seq2);
  CHECK(instance.is_tour());
  PartialSequenceSolution pss(&instance, std::vector<int>{0, 1, 2, 3, 4});
  CHECK(pss.get_trajectory().is_tour());
  CHECK(pss.is_feasible());
  auto traj2 = tour_lns(instance, pss.get_trajectory(), 1, 3);
  CHECK(traj2.length() <= pss.obj());
}

TEST_CASE("LNS Larger") {
  std::vector<Circle> circles;
  for (double x = 0.0; x < 15.0; x += 1.5) {
    for (double y = 0.0; y < 15.0; y += 1.5) {
      circles.emplace_back(Point{x, y}, 1);
    }
  }
  Instance instance{circles};
  auto sol = compute_tour_by_2opt(instance);
  sol.simplify();
  auto opt_sol = optimize_tour_by_lns(instance, sol.get_trajectory(), 20, 10);
  CHECK(opt_sol.length() <= doctest::Approx(sol.obj()));
}
} // namespace cetsp
#endif // CETSP_HEURISTICS_H
