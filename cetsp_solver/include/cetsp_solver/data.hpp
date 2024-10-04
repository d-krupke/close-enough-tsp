#pragma once
# include <vector>
# include <cmath>

namespace cetsp_solver {
class Point {
public:
  Point(double x, double y) : x(x), y(y) {}
  double x, y;

  bool operator==(const Point &p) const {
    return x == p.x && y == p.y;
  }

};

class Circle {
public:
    Circle() : center(Point(0, 0)), radius(0) {}
    Circle(const Point& center, double radius) : center(center), radius(radius) {}
    Point center;
    double radius;
};

class Segment {
public:
    Segment(const Point& p1, const Point& p2) : p1(p1), p2(p2) {}
    Point p1, p2;
};

using Instance = std::vector<Circle>;
using Trajectory = std::vector<Point>;

double squared_distance(const Point &p1, const Point &p2);
double distance(const Point& p1, const Point& p2);
double distance(const Circle &c1, const Circle &c2);
double distance(const Segment &s, const Point &p);
double distance(const Segment &s, const Circle &p);
double distance(const Trajectory &t, const Point &p);

struct Solution {
    Solution(const Trajectory& trajectory, double cost) : trajectory(trajectory), cost(cost) {}
    Solution(const Trajectory& trajectory) : trajectory(trajectory) {
        cost = 0;
        for(u_int64_t i = 0; i < trajectory.size(); i++) {
            cost += distance(trajectory[i], trajectory[(i + 1) % trajectory.size()]);
        }
    }
    Solution() : cost(0) {}

    Trajectory trajectory;
    double cost;
};

/**
 * @brief Compute the closest point on a segment to a point.
 * This point may be the end of the segment or a point on the segment.
 * Tiny rounding errors may cause the point to be slightly outside the segment.
 * 
 * @param s The segment on which the closest point should be found.
 * @param p The reference point.
 * @return Point The closest point on the segment to the reference point.
 */
std::pair<Point, double> closest_point(const Segment &s, const Point &p);

} // namespace cetsp_solver
