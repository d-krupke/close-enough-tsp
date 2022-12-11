//
// Created by Dominik Krupke on 11.12.22.
//

#ifndef CETSP_HEURISTICS_H
#define CETSP_HEURISTICS_H
#include "cetsp/common.h"
#include "doctest/doctest.h"
namespace cetsp{
Trajectory compute_tour_by_2opt(const std::vector<Circle>& circles);

TEST_CASE("2Opt") {
  std::vector<Circle> seq  = {{{0,0}, 1}, {{1,1}, 1}, {{1,0}, 1}, {{0,1}, 1}};
  auto traj = compute_tour_by_2opt(seq);
  CHECK(traj.length() == doctest::Approx(4));
}
}
#endif // CETSP_HEURISTICS_H
