//
// Created by Dominik Krupke on 11.12.22.
//

#include "cetsp/common.h"
#include "cetsp/soc.h"
namespace cetsp {

bool swap_improves(std::vector<Circle> &circles, int i, int j) {
  int prev_i = (i==0? circles.size()-1: i-1);
  int next_j = (j+1)% circles.size();
  const auto prev_dist = circles[i].center.dist(circles[prev_i].center) + circles[j].center.dist(circles[next_j].center);
  const auto new_dist = circles[i].center.dist(circles[next_j].center) + circles[j].center.dist(circles[prev_i].center);
  return new_dist < prev_dist;
}

Trajectory compute_tour_by_2opt(std::vector<Circle> circles, bool path=false) {

  bool changed = true;
  const auto n = circles.size();
  while (changed) {
    changed = false;
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < i; j++) {
        if (swap_improves(circles, i, j)) {
          std::reverse(circles.begin() + j, circles.begin() + i+1);
          changed = true;
        }
      }
    }
  }
  return compute_tour(circles, path);
}
} // namespace cetsp