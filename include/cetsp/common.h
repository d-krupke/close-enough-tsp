//
// Created by Dominik Krupke on 11.12.22.
//

#ifndef CLOSE_ENOUGH_TSP_COMMON_H
#define CLOSE_ENOUGH_TSP_COMMON_H
#include "cetsp/cgal_kernel.h"
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

class Trajectory {
  /**
   * For representing the trajectory in a solution.
   */
public:
  Trajectory(){};
  Trajectory(std::vector<Point> points) : points{std::move(points)} {}

  bool is_tour() const { return points[0] == points[points.size() - 1]; }

  double distance(const Circle &circle) const {
    double min_dist = std::numeric_limits<double>::infinity();
    details::Point p(circle.center.x, circle.center.y);
    if (points.size() == 1) {
      details::Point tp(points[0].x, points[0].y);
      min_dist = CGAL::squared_distance(tp, p);
    }
    for (int i = 0; i < points.size() - 1; i++) {
      details::Segment segment({points[i].x, points[i].y},
                               {points[i + 1].x, points[i + 1].y});
      double dist = CGAL::squared_distance(segment, p);
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
    return std::sqrt(min_dist) - circle.radius;
  }

  double length() const {
    double l = 0;
    for (int i = 0; i < points.size() - 1; i++) {
      details::Segment segment({points[i].x, points[i].y},
                               {points[i + 1].x, points[i + 1].y});
      l += std::sqrt(segment.squared_length());
    }
    return l;
  }

  bool covers(const Circle &circle) const { return distance(circle) <= 0.0; }

  std::vector<Point> points;
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