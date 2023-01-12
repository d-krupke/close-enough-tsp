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

Trajectory tour_lns(const Instance& instance, const std::vector<int>& sequence, const Trajectory& trajectory, int begin, int end) {
  assert(sequence.size() == trajectory.points.size()-1);
  const auto lns_path_begin = trajectory.points[begin];
  const auto lns_path_end = trajectory.points[end];
  auto fixed_trajectory = trajectory.sub(end, begin);
  Instance missing_circles;
  for(const auto& circle:  instance){
    if(!fixed_trajectory.covers(circle)) {
      missing_circles.push_back(circle);
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
    return trajectory;
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
  Trajectory traj{{Point{0, 0}, Point{3, 0}, Point{5, 2}, Point{3, 3}, Point{0, 4}, Point{0,0}}};
  CHECK(traj.length() >= 1);
  auto traj2 = tour_lns(instance, {0,1,2,3,4},traj, 1,4);
  CHECK(traj2.length()<=traj.length());
}
} // namespace cetsp
#endif // CETSP_HEURISTICS_H
