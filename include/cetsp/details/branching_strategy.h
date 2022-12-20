//
// Created by Dominik Krupke on 14.12.22.
//

#ifndef CETSP_BRANCHING_STRATEGY_H
#define CETSP_BRANCHING_STRATEGY_H
#include "cetsp/common.h"
#include "cetsp/node.h"
#include <vector>

#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>
namespace cetsp {
class BranchingStrategy {
  virtual bool branch(Node &node) = 0;
};

class FarthestCircle : BranchingStrategy {
public:
  FarthestCircle(Instance *instance) : instance{instance} {}
  bool branch(Node &node) {
    std::vector<double> distances(instance->size());
    for (unsigned i = 0; i < instance->size(); ++i) {
      distances[i] = node.get_relaxed_solution().distance((*instance)[i]);
    }
    auto max_dist = std::max_element(distances.begin(), distances.end());
    if (*max_dist <= 0) {
      return false;
    }
    const int c = std::distance(distances.begin(), max_dist);

    std::vector<Node> children;
    std::vector<int> seqeuence = node.get_fixed_sequence();
    seqeuence.push_back(c);
    if (instance->is_path()) {
      // for path, this position may not be symmetric.
      children.emplace_back(seqeuence, instance, &node);
    }
    for (int i = seqeuence.size() - 1; i > 0; --i) {
      seqeuence[i] = seqeuence[i - 1];
      seqeuence[i - 1] = c;
      children.emplace_back(seqeuence, instance, &node);
    }
    node.branch(std::move(children));
    return true;
  }

private:
  Instance *instance;
};

class ChFarthestCircle : BranchingStrategy {
public:
  ChFarthestCircle(Instance *instance) : instance{instance} {
    if (instance->is_path()) {
      throw std::invalid_argument(
          "ConvexHull Strategy only feasible for tours.");
    }
    order_values.resize(instance->size());
    is_ordered.resize(instance->size(), false);
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point_2;
    typedef K::Segment_2 Segment_2;
    typedef K::Ray_2 Ray_2;
    typedef K::Direction_2 Direction_2;
    typedef CGAL::Convex_hull_traits_adapter_2<
        K, CGAL::Pointer_property_map<Point_2>::type>
        Convex_hull_traits_2;
    std::vector<Point_2> points;
    points.reserve(instance->size());
    for (const auto &c : *instance) {
      points.push_back({c.center.x, c.center.y});
    }
    std::vector<int> indices(points.size()), out;
    std::iota(indices.begin(), indices.end(), 0);
    CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                        Convex_hull_traits_2(CGAL::make_property_map(points)));
    std::vector<Segment_2> ch_segments;
    for (int i = 0; i < out.size(); ++i) {
      ch_segments.emplace_back(points[out[i]],
                               points[out[(i + 1) % out.size()]]);
    }
    for (int i = 0; i < instance->size(); ++i) {
      double weight = 0.0;
      const auto &p = points[i];
      double squared_radius = instance->at(i).radius * instance->at(i).radius;
      for (const auto &s : ch_segments) {
        if (squared_distance(s, points[i]) <= squared_radius) {
          // segment in range
          Ray_2 r1{s.source(), Direction_2{-(s.target().y() - s.source().y()),
                                           (s.target().x() - s.source().x())}};
          Ray_2 r2{s.source(), Direction_2{-(s.target().y() - s.source().y()),
                                           (s.target().x() - s.source().x())}};
          if (squared_distance(r1, p) <= s.squared_length() and
              squared_distance(r2, p) <= s.squared_length()) {
            // segment between both rays
            weight +=
                std::sqrt(squared_distance(r1, p)); // add distance to first ray
            is_ordered[i] = true;
            order_values[i] = weight;
            break;
          }
        }
        weight += std::sqrt(s.squared_length());
      }
    }
  }

  bool sequence_is_ch_ordered(const std::vector<int> &seqeuence) {
    double weight = 0;
    for (auto j: seqeuence) {
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

  bool branch(Node &node) {
    if (!verified_root) {
      if (!sequence_is_ch_ordered(node.get_fixed_sequence())) {
        for(auto i: node.get_fixed_sequence()) {
          std::cout  << i <<"\t"<<is_ordered[i]<<"\t"<<order_values[i] <<std::endl;
        }
        throw std::invalid_argument(
            "Current sequence does not obey the order.");
      }
      verified_root = true;
    }
    std::vector<double> distances(instance->size());
    for (unsigned i = 0; i < instance->size(); ++i) {
      distances[i] = node.get_relaxed_solution().distance((*instance)[i]);
    }
    auto max_dist = std::max_element(distances.begin(), distances.end());
    if (*max_dist <= 0) {
      return false;
    }
    const int c = std::distance(distances.begin(), max_dist);

    std::vector<Node> children;
    std::vector<int> seqeuence = node.get_fixed_sequence();
    seqeuence.push_back(c);
    if (instance->is_path()) {
      // for path, this position may not be symmetric.
      children.emplace_back(seqeuence, instance, &node);
    }
    for (int i = seqeuence.size() - 1; i > 0; --i) {
      seqeuence[i] = seqeuence[i - 1];
      seqeuence[i - 1] = c;

      if (!sequence_is_ch_ordered(seqeuence)) {
        continue;
      }
      children.emplace_back(seqeuence, instance, &node);
    }
    node.branch(std::move(children));
    return true;
  }

private:
  Instance *instance;
  std::vector<double> order_values;
  std::vector<bool> is_ordered;
  bool verified_root = false;
};

TEST_CASE("Branching Strategy") {
  // The strategy should choose the triangle and implicitly cover the
  // second circle.
  std::vector<Circle> instance_ = {
      {{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}, {{3, 6}, 1}};
  Instance instance(instance_);
  FarthestCircle bs(&instance);
  Node root({0, 1, 2, 3}, &instance);
  CHECK(bs.branch(root) == false);

  std::vector<Circle> seq = {{{0, 0}, 1}, {{3, 0}, 1}, {{6, 0}, 1}};
  Node root2({0, 1, 2}, &instance);
  CHECK(bs.branch(root2) == true);
  CHECK(root2.get_children().size() == 3);
}
} // namespace cetsp
#endif // CETSP_BRANCHING_STRATEGY_H
