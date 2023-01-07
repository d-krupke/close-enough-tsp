//
// Provides some common classes on instances and solutions that are
// independent of the concrete algorithms.
//

#ifndef CLOSE_ENOUGH_TSP_COMMON_H
#define CLOSE_ENOUGH_TSP_COMMON_H
#include "cetsp/details/cgal_kernel.h"
#include "doctest/doctest.h"
#include <CGAL/squared_distance_2.h> //for 2D functions
#include <cmath>

namespace cetsp {
class Point {
  /**
   * Represents a single coordinate.
   */
public:
  Point() {}
  Point(double x, double y) : x{x}, y{y} {}
  double x;
  double y;

  double dist(const Point &point) const {
    return std::sqrt(squared_dist(point));
  }

  double squared_dist(const Point &point) const {
    return (point.x - x) * (point.x - x) + (point.y - y) * (point.y - y);
  }

  bool operator==(const Point &point) const {
    return point.x == x && point.y == y;
  }
  bool operator!=(const Point &point) const { return !((*this) == point); }
};

TEST_CASE("Point") {
  Point p1(0, 0);
  Point p2(2, 0);
  CHECK(p1 != p2);
  CHECK(p1 == p1);
  CHECK(p1.dist(p2) == 2.0);
  CHECK(p1.squared_dist(p2) == 4.0);
}

class Circle {
  /**
   * Represents a circle, consisting of a center and a radius.
   */
public:
  Circle(){};
  Circle(Point center, double radius) : center{center}, radius{radius} {}

  bool contains(const Point &point) const {
    return center.squared_dist(point) <= radius * radius;
  }

  bool contains(const Circle &circle) const {
    return center.dist(circle.center) + circle.radius <= 1.001 * radius;
  }

  Point center;
  double radius;
};

TEST_CASE("Circle") {
  Circle c1(Point(0, 0), 1);
  Circle c2(Point(0, 0), 0.5);
  Point p(1, 0);
  CHECK(c1.contains(p));
  CHECK(!c2.contains(p));
}

class Instance : public std::vector<Circle> {
public:
  Instance() {}
  Instance(std::vector<Circle> circles) {
    reserve(circles.size());
    std::sort(circles.begin(), circles.end(),
              [](const auto &a, const auto &b) { return a.radius < b.radius; });
    for (const auto &circle : circles) {
      if (std::any_of(begin(), end(),
                      [&circle](auto &c) { return circle.contains(c); })) {
        std::cout << "Removed implicit circle (" << circle.center.x << ", "
                  << circle.center.y << std::endl;
        continue;
      }
      push_back(circle);
    }
  }
  bool is_path() const {
    if (path) {
      return true;
    } else {
      return false;
    }
  }

  bool is_tour() const {
    if (path) {
      return false;
    }
    { return true; }
  }

  void add_circle(Circle &circle) {
    if (std::any_of(begin(), end(),
                    [&circle](auto &c) { return circle.contains(c); })) {
      return;
    }
    push_back(circle);
    revision += 1;
  }

  std::optional<std::pair<Point, Point>> path;
  int revision = 0;
  double eps = 0.01;
};

class Trajectory {
  /**
   * For representing the trajectory in a solution.
   */
public:
  Trajectory() = default;
  explicit Trajectory(std::vector<Point> points) : points{std::move(points)} {}

  bool is_tour() const { return points[0] == points[points.size() - 1]; }

  double distance(const Circle &circle) const {
    double min_dist = std::numeric_limits<double>::infinity();
    details::cgPoint p(circle.center.x, circle.center.y);
    if (points.size() == 1) {
      details::cgPoint tp(points[0].x, points[0].y);
      min_dist = CGAL::squared_distance(tp, p);
    }
    for (unsigned i = 0; i < points.size() - 1; i++) {
      details::cgSegment segment({points[i].x, points[i].y},
                                 {points[i + 1].x, points[i + 1].y});
      double dist = CGAL::squared_distance(segment, p);
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
    return std::sqrt(min_dist) - circle.radius;
  }

  double length() const {
    if (!_length) {
      double l = 0;
      for (unsigned i = 0; i < points.size() - 1; i++) {
        details::cgSegment segment({points[i].x, points[i].y},
                                   {points[i + 1].x, points[i + 1].y});
        l += std::sqrt(segment.squared_length());
      }
      _length = l;
    }
    return *_length;
  }

  bool is_simple() const {
    std::vector<details::cgPoint> points_;
    for (const auto &p : points) {
      if (!points_.empty() &&
          p.dist(Point(points_.back().x(), points_.back().y())) < 0.01) {
        continue;
      }
      points_.emplace_back(p.x, p.y);
    }
    if (points.front() == points.back()) {
      details::cgPolygon poly{points_.begin(), points_.end() - 1};
      return poly.is_simple();
    } else {
      std::cout << "Warning! `is_simple`  does not work for paths right now!"
                << std::endl;
      details::cgPolygon poly{points_.begin(), points_.end()};
      return poly.is_simple();
    }
  }

  [[nodiscard]] bool covers(const Circle &circle, double eps = 0.0) const {
    return distance(circle) <= eps;
  }

  template <typename It>
  [[nodiscard]] auto covers(It begin, It end, double eps = 0.0) const -> bool {
    return std::all_of(begin, end,
                       [&](const Circle &c) { return this->covers(c, eps); });
  }

  std::vector<Point> points;

private:
  mutable std::optional<double> _length;
};

TEST_CASE("Trajectory") {
  Trajectory traj{{{0, 0}, {5, 0}, {5, 5}}};
  CHECK(!traj.is_tour());
  Circle c1({0, 0}, 1);
  CHECK(traj.distance(c1) == -1);
  CHECK(traj.covers(c1));
  CHECK(traj.length() == 10.0);
}
}; // namespace cetsp

#endif // CLOSE_ENOUGH_TSP_COMMON_H
