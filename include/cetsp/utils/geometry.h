//
// Created by Dominik Krupke on 15.01.23.
//

#ifndef CETSP_GEOMETRY_H
#define CETSP_GEOMETRY_H
#include "doctest/doctest.h"
#include <cmath>
#include <utility>
namespace cetsp::utils {
double distance_to_segment(std::pair<double, double> s0,
                           std::pair<double, double> s1,
                           std::pair<double, double> p);

TEST_CASE("Distance to Segment") {
  CHECK(distance_to_segment({0, 0}, {10, 0}, {0, 0}) == doctest::Approx(0.0));
  CHECK(distance_to_segment({0, 0}, {10, 0}, {0, 1}) == doctest::Approx(1.0));
  CHECK(distance_to_segment({0, 0}, {10, 0}, {0, -1}) == doctest::Approx(1.0));
  CHECK(distance_to_segment({0, 0}, {10, 0}, {-1, 0}) == doctest::Approx(1.0));
  CHECK(distance_to_segment({0, 0}, {10, 0}, {11, 0}) == doctest::Approx(1.0));
}
} // namespace cetsp::utils

#endif // CETSP_GEOMETRY_H
