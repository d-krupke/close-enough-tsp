#include "cetsp_solver/data.hpp"

namespace cetsp_solver {

double squared_distance(const Point &p1, const Point &p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) +
         (p1.y - p2.y) * (p1.y - p2.y);
}    
double distance(const Point &p1, const Point &p2) {
  return std::sqrt(squared_distance(p1, p2));
}

double distance(const Circle &c1, const Circle &c2) {
  return std::max(0.0, distance(c1.center, c2.center) - c1.radius - c2.radius);
}

double distance(const Segment &s, const Point &p) {
    // stolen from
    // https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/
    using namespace std;
    
    // vector AB
    double AB_x = s.p2.x - s.p1.x;
    double AB_y = s.p2.y - s.p1.y;

    // vector BP
    double BE_x = p.x - s.p2.x;
    double BE_y = p.y - s.p2.y;

    // vector AP
    double AE_x = p.x - s.p1.x;
    double AE_y = p.y - s.p1.y;

    // Variables to store dot product
    double AB_BE, AB_AE;

    // Calculating the dot product
    AB_BE = (AB_x * BE_x + AB_y * BE_y);
    AB_AE = (AB_x * AE_x + AB_y * AE_y);

    // Minimum distance from
    // point E to the line segment
    double reqAns = 0;

    // Case 1
    if (AB_BE > 0) {

        // Finding the magnitude
        double y = p.y - s.p2.y;
        double x = p.x - s.p2.x;
        reqAns = sqrt(x * x + y * y);
    }

    // Case 2
    else if (AB_AE < 0) {
        double y = p.y - s.p1.y;
        double x = p.x - s.p1.x;
        reqAns = sqrt(x * x + y * y);
    }

    // Case 3
    else {
        // Finding the perpendicular distance
        reqAns = std::abs(AB_x * AE_y - AB_y * AE_x) / std::sqrt(AB_x * AB_x + AB_y * AB_y);
    }
    return reqAns;
}
double distance(const Segment& segment, const Circle& circle) {
    return std::max(0.0, distance(segment, circle.center) - circle.radius);
}

double distance(const Trajectory &t, const Point &p) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (unsigned i = 0; i < t.size(); ++i) {
        Segment s(t[i], t[(i + 1) % t.size()]);
        min_dist = std::min(min_dist, distance(s, p));
    }
    return min_dist;
}

double distance(const Trajectory &t, const Circle &c) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (unsigned i = 0; i < t.size(); ++i) {
        Segment s(t[i], t[(i + 1) % t.size()]);
        min_dist = std::min(min_dist, distance(s, c));
    }
    return min_dist;
}


std::pair<Point, double> closest_point(const Segment &s, const Point &p) {
    if(s.p1 == s.p2) {
      // Trivial case: the segment is a point, return the point
        return {s.p1, 0};
    }
    // Compute the projection of the point on the line defined by the segment
    double t = ((p.x - s.p1.x) * (s.p2.x - s.p1.x) + (p.y - s.p1.y) * (s.p2.y - s.p1.y)) /
               (std::pow(s.p2.x - s.p1.x, 2) + std::pow(s.p2.y - s.p1.y, 2));
    // Clamp the projection to the segment
    t = std::max(0.0, std::min(1.0, t));
    return {Point(s.p1.x + t * (s.p2.x - s.p1.x), s.p1.y + t * (s.p2.y - s.p1.y)), t};
}

} // namespace cetsp_solver