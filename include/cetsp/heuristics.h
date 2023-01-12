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

Trajectory tour_lns(const Instance& instance, PartialSequenceSolution& solution, int begin, int end) {
  assert(instance.is_tour());
  solution.simplify();
  const auto lns_path_begin = solution.trajectory_begin();
  const auto lns_path_end = solution.trajectory_end();
  auto fixed_trajectory = solution.get_trajectory().sub(end, begin);
  Instance missing_circles;
    for(int  i=0;i<instance.size(); ++i){
    if(!solution.covers(i)) {
      missing_circles.push_back(instance[i]);
    }
  }
  missing_circles.path = {lns_path_begin, lns_path_end};
  // solve subinstance
  std::unique_ptr<RootNodeStrategy> rns;
  rns = std::make_unique<LongestEdgePlusFurthestCircle>();
  std::unique_ptr<BranchingStrategy> branching_strategy;
  branching_strategy = std::make_unique<ChFarthestCircle>(true);
  std::unique_ptr<SearchStrategy> search_strategy;
  search_strategy = std::make_unique<DfsBfs>();
  BranchAndBoundAlgorithm baba(&missing_circles, rns->get_root_node(missing_circles),
                               *branching_strategy, *search_strategy);
  baba.optimize(30);
  // build new solution
  auto sol =  baba.get_solution();
  if(!sol) {
    return solution.get_trajectory();
  }
  assert(!sol->is_tour());
  std::vector<Point> points;
  for(const auto&  p: fixed_trajectory.points){
    points.push_back(p);
  }
  for(const auto&  p: sol->points){
    if(points.back()!=p) {
      points.push_back(p);
    }
  }
  return Trajectory{points};
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
  PartialSequenceSolution pss(&instance, std::vector<int>{0,1,2,3,4});
  CHECK(pss.get_trajectory().is_tour());
  CHECK(pss.is_feasible());
  auto traj2 = tour_lns(instance, pss, 1,3);
  CHECK(traj2.length()<=pss.obj());
}
} // namespace cetsp
#endif // CETSP_HEURISTICS_H
