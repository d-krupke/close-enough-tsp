//
// Created by Dominik Krupke on 07.01.23.
//
#include "cetsp/details/convex_hull_order.h"
namespace cetsp {
namespace details {

std::optional<double> get_distance_on_segment(const Segment_2 &s,
                                              const Point_2 &p) {
  // segment in range
  Ray_2 r1{s.source(), Direction_2{-(s.target().y() - s.source().y()),
                                   (s.target().x() - s.source().x())}};
  Ray_2 r2{s.target(), Direction_2{-(s.target().y() - s.source().y()),
                                   (s.target().x() - s.source().x())}};
  if (squared_distance(r1, p) <= s.squared_length() &&
      squared_distance(r2, p) <= s.squared_length()) {
    // segment between both rays
    return {std::sqrt(squared_distance(r1, p))}; // add distance to first ray
  }
  return {};
}

std::optional<double> ConvexHullOrder::operator()(const Circle &circle) {
  const Point_2 p{circle.center.x, circle.center.y};
  const double radius = circle.radius;
  double weight = 0.0;
  for (const auto &segment : segments) {
    if (squared_distance(segment, p) > radius * radius) {
      // not intersecting segment
      weight += std::sqrt(segment.squared_length());
      continue;
    }
    auto val = get_distance_on_segment(segment, p);
    if (val) {
      return weight + (*val);
    } else {
      weight += std::sqrt(segment.squared_length());
    }
  }
  return {};
}

std::vector<Segment_2> ConvexHullOrder::compute_convex_hull_segments(
    const std::vector<Point> &points) const {
  /**
   * Compute the segments on the convex hull, ordered counter-clockwise.
   */
  std::vector<Point_2> points_;
  points_.reserve(points.size());
  for (const auto &p : points) {
    points_.emplace_back(p.x, p.y);
  }
  std::vector<int> indices(points_.size()), out;
  std::iota(indices.begin(), indices.end(), 0);
  CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                      Convex_hull_traits_2(CGAL::make_property_map(points_)));
  std::vector<Segment_2> ch_segments;
  for (unsigned i = 0; i < out.size(); ++i) {
    ch_segments.emplace_back(points_[out[i]],
                             points_[out[(i + 1) % out.size()]]);
  }
  return ch_segments;
}
} // namespace details
} // namespace cetsp
