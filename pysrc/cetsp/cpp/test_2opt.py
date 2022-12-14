from ._cetsp_cpp import Point, Circle, compute_tour_by_2opt, Trajectory


def test_2opt():
    circles = [Circle(Point(x, y), 1) for x in range(10) for y in range(10)]
    traj = compute_tour_by_2opt(circles, False)
