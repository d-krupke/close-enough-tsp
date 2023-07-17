/**
 * In this test, we want to check the lazy callback capability of our approach.
 * Note that the lazy constraints should only add circles within the convex
 * hull. Otherwise, the result may be wrong.
 *
 * Note that it is best to start with a feasible solution, which of course
 * has to satisfy all lazy constraints.
 */

#ifndef CETSP_LAZY_CALLBACK_TESTS_H
#define CETSP_LAZY_CALLBACK_TESTS_H

#include "cetsp/bnb.h"
#include "cetsp/callbacks.h"
#include "doctest/doctest.h"

class LazyCB : public cetsp::B2BNodeCallback {
public:
  LazyCB(std::vector<cetsp::Circle> circles) : circles{std::move(circles)} {}
  virtual void add_lazy_constraints(cetsp::EventContext &e) {
    for (auto &c : circles) {
      if (!e.get_relaxed_solution().get_trajectory().covers(c, 0.001)) {
        // it is stupid to just add a random  circle, but fine enough for
        // testing.
        e.add_lazy_circle(c);
        std::cout << "Add circle." << std::endl;
        return;
      }
    }
  }

private:
  std::vector<cetsp::Circle> circles;
};

TEST_CASE("Lazy Callback") {
  std::vector<cetsp::Circle> circles;
  for (double x = 0; x <= 10; x += 2.0) {
    for (double y = 0; y <= 10; y += 2.0) {
      circles.push_back({{x, y}, 1});
    }
  }
  cetsp::Instance instance(cetsp::Instance(
      {{{0, 0}, 1}, {{10, 0}, 1}, {{10, 10}, 1}, {{0, 10}, 1}}));
  cetsp::ConvexHullRoot root_node_strategy{};
  cetsp::FarthestCircle branching_strategy{true, 8};
  cetsp::DfsBfs search_strategy;
  cetsp::BranchAndBoundAlgorithm bnb(&instance,
                                     root_node_strategy.get_root_node(instance),
                                     branching_strategy, search_strategy);
  bnb.add_node_callback(std::make_unique<LazyCB>(circles));
  bnb.optimize(30, 0.01);
  CHECK(bnb.get_solution());
  CHECK(bnb.get_solution()->get_trajectory().covers(circles.begin(),
                                                    circles.end(), 0.001));
  CHECK(bnb.get_upper_bound() <= 41.0);
  CHECK(bnb.get_lower_bound() >= 39.0);
}

#endif // CETSP_LAZY_CALLBACK_TESTS_H
