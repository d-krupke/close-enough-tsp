from ._cetsp_bindings import Point, Circle, compute_tour_by_2opt, Trajectory, branch_and_bound, Instance


def test_2opt():
    circles = [Circle(Point(x, y), 1) for x in range(10) for y in range(10)]
    instance = Instance(circles)
    traj = compute_tour_by_2opt(instance)
    #traj = compute_tour_by_2opt(circles, False)

def test_bnb():
    circles = [Circle(Point(x, y), 1) for x in range(5) for y in range(5)]
    instance = Instance(circles)
    instance.circles()
    traj = compute_tour_by_2opt(instance)

    def bla(node, solution_pool):
        #node.prune()
        solution_pool.add_solution(traj)
        print("py", node.get_lower_bound(), solution_pool.get_upper_bound())
    traj = branch_and_bound(instance, bla )

