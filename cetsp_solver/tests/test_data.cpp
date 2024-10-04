#include <gtest/gtest.h>
#include "cetsp_solver/data.hpp"

TEST(PointTest, DistanceTest) {
    using namespace cetsp_solver;
    Point p1(2, 4);
    Point p2(4, 4);
    auto dist = distance(p1, p2);
    EXPECT_NEAR(dist, 2.0, 0.001);
}

TEST(CircleTest, DistanceTest) {
    using namespace cetsp_solver;
    Circle c1(Point(0, 0), 1);
    Circle c2(Point(3, 0), 1);
    auto dist = distance(c1, c2);
    EXPECT_NEAR(dist, 1.0, 0.001);
}

TEST(SegmentTest, DistanceTest1) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(3, 0));
    Point p(1, 1);
    auto dist = distance(s, p);
    EXPECT_NEAR(dist, 1.0, 0.001);
}

TEST(SegmentTest, DistanceTest2) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(3, 0));
    Point p(1, 0);
    auto dist = distance(s, p);
    EXPECT_NEAR(dist, 0.0, 0.001);
}

TEST(SegmentTest, DistanceTest3) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(3, 0));
    Point p(4, 0);
    auto dist = distance(s, p);
    EXPECT_NEAR(dist, 1.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest1) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(3, 0));
    Point p(1, 1);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 1.0, 0.001);
    EXPECT_NEAR(cp.y, 0.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest2) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(3, 0));
    Point p(1, 0);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 1.0, 0.001);
    EXPECT_NEAR(cp.y, 0.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest3) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(3, 0));
    Point p(4, 0);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 3.0, 0.001);
    EXPECT_NEAR(cp.y, 0.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest4) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(0, 3));
    Point p(1, 1);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 0.0, 0.001);
    EXPECT_NEAR(cp.y, 1.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest5) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(0, 3));
    Point p(0, 1);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 0.0, 0.001);
    EXPECT_NEAR(cp.y, 1.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest6) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(0, 3));
    Point p(0, 4);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 0.0, 0.001);
    EXPECT_NEAR(cp.y, 3.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest7) {
    using namespace cetsp_solver;
    Segment s(Point(0, 0), Point(0, 3));
    Point p(1, 4);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 0.0, 0.001);
    EXPECT_NEAR(cp.y, 3.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest8) {
    using namespace cetsp_solver;
    Segment s(Point(3, 3), Point(3, 10));
    Point p(1, 4);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 3.0, 0.001);
    EXPECT_NEAR(cp.y, 4.0, 0.001);
}

TEST(ClosestPointTest, SegmentTest9) {
    using namespace cetsp_solver;
    Segment s(Point(3, 3), Point(3, 10));
    Point p(4, 1);
    auto [cp, t] = closest_point(s, p);
    EXPECT_NEAR(cp.x, 3.0, 0.001);
    EXPECT_NEAR(cp.y, 3.0, 0.001);
}
