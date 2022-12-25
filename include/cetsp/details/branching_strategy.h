/**
 * This file implements branching stratgies for the branch and bound algorithm,
 * i.e., deciding how to split  the solution space. The primary decision here
 * is to decide for the circle to integrate next. However, we can also do
 * some filtering here and only create branches  that are promising.
 *
 * The simplest branching strategy is to use always the furthest circle to
 * the current relaxed solution.
 */
#ifndef CETSP_BRANCHING_STRATEGY_H
#define CETSP_BRANCHING_STRATEGY_H
#include "cetsp/common.h"
#include "cetsp/details/solution_pool.h"
#include "cetsp/details/triple_map.h"
#include "cetsp/node.h"
#include <vector>

#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>
namespace cetsp {
class BranchingStrategy {
public:
  virtual void setup(Instance *instance, Node *root,
                     SolutionPool *solution_pool) {}
  virtual bool branch(Node &node) = 0;
};

class FarthestCircle : public BranchingStrategy {
  /**
   * This strategy tries to branch on the circle that is most distanced
   * to the relaxed solution.
   */
public:
  FarthestCircle() {}

  virtual void setup(Instance *instance_, Node *root,
                     SolutionPool *solution_pool) override {
    instance = instance_;
  }

  bool branch(Node &node) override;

  virtual bool allows_lazy_constraints() { return true; }

protected:
  /**
   * Override this method to filter the branching in advance.
   * @param sequence Sequence to be checked for a potential branch.
   * @return True if branch should be created.
   */
  virtual bool is_sequence_ok(const std::vector<int> &sequence) {
    // the base version will accept every sequence. Inherit and override
    // to change this behavior.
    return true;
  }
  Instance *instance = nullptr;
};

class ChFarthestCircle : public FarthestCircle {
  /**
   * This strategy will only create branches that satisfy the CCW order of the
   * convex hull. A dependency is that the root is also obeying this rule.
   * We can proof that any optimal solution has follow the order of circles
   * intersecting the convex hull on the circle centers.
   *
   * This does not allow lazy constraints! (or only those that do not change
   * the convex hull).
   */
public:
  virtual bool allows_lazy_constraints() override { return false; }

  virtual void setup(Instance *instance_, Node *root,
                     SolutionPool *solution_pool_) override {
    FarthestCircle::setup(instance_, root, solution_pool_);
    solution_pool = solution_pool_;
    if (instance->is_path()) {
      // Should be possible to extend the idea to paths.
      throw std::invalid_argument(
          "ConvexHull Strategy only feasible for tours.");
    }
    order_values.resize(instance->size());
    is_ordered.resize(instance->size(), false);
    compute_weights(instance, root);
  }

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point_2;
  typedef K::Segment_2 Segment_2;
  typedef K::Ray_2 Ray_2;
  typedef K::Direction_2 Direction_2;
  typedef CGAL::Convex_hull_traits_adapter_2<
      K, CGAL::Pointer_property_map<Point_2>::type>
      Convex_hull_traits_2;

  /**
   * Compute the CCW segments of the convex hull of a set of points.
   * @param points The points to compute the CH on.
   * @return CCW ordered segements representing the CH.
   */
  std::vector<Segment_2>
  compute_convex_hull_segments(std::vector<Point_2> &points) const {
    std::vector<int> indices(points.size()), out;
    std::iota(indices.begin(), indices.end(), 0);
    CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                        Convex_hull_traits_2(CGAL::make_property_map(points)));
    std::vector<Segment_2> ch_segments;
    for (unsigned i = 0; i < out.size(); ++i) {
      ch_segments.emplace_back(points[out[i]],
                               points[out[(i + 1) % out.size()]]);
    }
    return ch_segments;
  }

  std::vector<Point_2> get_circle_centers(Instance &instance) const {
    std::vector<Point_2> points;
    points.reserve(instance.size());
    for (const auto &c : instance) {
      points.push_back({c.center.x, c.center.y});
    }
    return points;
  }

  std::optional<double> get_distance_on_segment(const Segment_2 &s,
                                                const Point_2 &p) const {

    // segment in range
    Ray_2 r1{s.source(), Direction_2{-(s.target().y() - s.source().y()),
                                     (s.target().x() - s.source().x())}};
    Ray_2 r2{s.source(), Direction_2{-(s.target().y() - s.source().y()),
                                     (s.target().x() - s.source().x())}};
    if (squared_distance(r1, p) <= s.squared_length() &&
        squared_distance(r2, p) <= s.squared_length()) {
      // segment between both rays
      return {std::sqrt(squared_distance(r1, p))}; // add distance to first ray
    }
    return {};
  }

  void compute_weights(Instance *instance, Node *root) {
    auto points = get_circle_centers(*instance);
    auto ch_segments = compute_convex_hull_segments(points);
    for (unsigned i = 0; i < instance->size(); ++i) {
      double weight = 0.0;
      const auto &p = points[i];
      double squared_radius = instance->at(i).radius * instance->at(i).radius;
      for (const auto &s : ch_segments) {
        if (squared_distance(s, points[i]) <= squared_radius) {
          // segment in range
          auto w = get_distance_on_segment(s, p);
          if (w) {
            weight += *w;
            is_ordered[i] = true;
            order_values[i] = weight;
            break;
          }
        }
        weight += std::sqrt(s.squared_length());
      }
    }
  }

  ChFarthestCircle() : FarthestCircle(), tm{instance} {}

protected:
  bool sequence_is_ch_ordered(const std::vector<int> &seqeuence) {
    double weight = 0;
    for (auto j : seqeuence) {
      if (is_ordered[j]) {
        if (order_values[j] < 0.999 * weight) {
          return false;
        } else {
          weight = order_values[j];
        }
      }
    }
    return true;
  }
  virtual bool is_sequence_ok(const std::vector<int> &sequence) override {
    if (tm.estimate_cost_for_sequence(sequence) >=
        solution_pool->get_upper_bound()) {
      std::cout << "early  pruning" << std::endl;
      return false;
    }
    return sequence_is_ch_ordered(sequence);
  }

private:
  TripleMap tm;
  std::vector<double> order_values;
  std::vector<bool> is_ordered;
  SolutionPool *solution_pool;
};

TEST_CASE("Branching Strategy") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  std::vector<Circle> instance_ = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  Instance instance(instance_);
  FarthestCircle bs;
  Node root({0, 1, 2, 3}, &instance);
  bs.setup(&instance, &root, nullptr);
  CHECK(bs.branch(root) == false);

  std::vector<Circle> seq = {{{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}};
  Node root2({0, 1, 2}, &instance);
  CHECK(bs.branch(root2) == true);
  CHECK(root2.get_children().size() == 3);
}
} // namespace cetsp
#endif // CETSP_BRANCHING_STRATEGY_H
