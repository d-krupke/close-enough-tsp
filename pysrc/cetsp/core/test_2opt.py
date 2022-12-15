from ._cetsp_bindings import Point, Circle, compute_tour_by_2opt, Trajectory, branch_and_bound, Instance


def test_2opt():
    circles = [Circle(Point(x, y), 1) for x in range(10) for y in range(10)]
    traj = compute_tour_by_2opt(circles, False)

def test_bnb():
    circles = [Circle(Point(x, y), 1) for x in range(5) for y in range(5)]
    instance = Instance(circles)
    instance.circles()
    def bla(node, solution_pool):
        print("py", node.get_lower_bound(), solution_pool.get_upper_bound())
    traj = branch_and_bound(instance, bla )

