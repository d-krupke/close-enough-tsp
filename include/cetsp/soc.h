#ifndef CETSP_SOC_H
#define CETSP_SOC_H
#include "cetsp/common.h"
#include "doctest/doctest.h"
#include <vector>
namespace cetsp {

Trajectory compute_tour(const std::vector<Circle> &circle_sequence,
                        const bool path = false);

TEST_CASE("SOCP") {
  std::vector<Circle> seq = {{{0, 0}, 1}, {{3, 0}, 1}};
  auto traj = compute_tour(seq, false);
  CHECK(traj.length() == doctest::Approx(2));
  traj = compute_tour(seq, true);
  CHECK(traj.length() == doctest::Approx(1));
}
} // namespace cetsp
#endif