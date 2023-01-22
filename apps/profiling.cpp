//
// Created by Dominik Krupke on 09.01.23.
//
#include "cetsp/bnb.h"
#include "cetsp/common.h"
#include "cetsp/heuristics.h"

int main() {
  using namespace cetsp;
  int timelimit = 300;
  std::vector<Circle> circles;
  for (double x = 0; x < 10; x += 1.1) {
    for (double y = 0; y < 10; y += 1.1) {
      circles.emplace_back(Point{x, y}, 1.3);
    }
  }
  Instance instance{circles};
  std::unique_ptr<RootNodeStrategy> rns;
  rns = std::make_unique<ConvexHullRoot>();
  std::unique_ptr<BranchingStrategy> branching_strategy;
  branching_strategy = std::make_unique<ChFarthestCircle>(true, 4);
  std::unique_ptr<SearchStrategy> search_strategy;
  search_strategy = std::make_unique<DfsBfs>();
  BranchAndBoundAlgorithm baba(&instance, rns->get_root_node(instance),
                               *branching_strategy, *search_strategy);
  auto initial_solution = compute_tour_by_2opt(instance);
  baba.add_upper_bound(initial_solution);
  baba.optimize(timelimit);
}
