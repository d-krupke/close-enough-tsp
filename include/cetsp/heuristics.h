//
// Created by Dominik Krupke on 11.12.22.
//

#ifndef CETSP_HEURISTICS_H
#define CETSP_HEURISTICS_H
#include "cetsp/common.h"
#include "doctest/doctest.h"
namespace cetsp{
Trajectory compute_tour_by_2opt(std::vector<Circle> circles, bool path=false);

TEST_CASE("2Opt") {
  std::vector<Circle> seq  = {{{0,0}, 0}, {{1,1}, 0}, {{1,0}, 0}, {{0,1}, 0}};
  auto traj = compute_tour_by_2opt(seq);
  CHECK(traj.length() == doctest::Approx(4));

  std::vector<Circle> seq2 = {{{0,0}, 1}, {{3,0}, 1}, {{5, 2}, 1}, {{3,3}, 1}, {{0, 4}, 1}};
  traj = compute_tour_by_2opt(seq2);
  CHECK(traj.length() >= 1);
}
}
#endif // CETSP_HEURISTICS_H
