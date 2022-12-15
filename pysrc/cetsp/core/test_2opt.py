from ._cetsp_bindings import Point, Circle, compute_tour_by_2opt, Trajectory, branch_and_bound, Instance


def test_2opt():
    circles = [Circle(Point(x, y), 1) for x in range(10) for y in range(10)]
    instance = Instance(circles)
    traj = compute_tour_by_2opt(instance)
    #traj = compute_tour_by_2opt(circles, False)

def test_bnb():
    circles = [Circle(Point(x, y), 1) for x in range(7) for y in range(7)]
    instance = Instance(circles)
    instance.circles()
    initial_solution = compute_tour_by_2opt(instance)

    def cb(node, solution_pool):
        #node.prune()
        solution_pool.add_solution(initial_solution)
        lb = node.get_lower_bound()
        ub = solution_pool.get_upper_bound()
        if(lb > 0.95*ub):
            node.prune()  # don't evaluate further
        #print("py", node.get_lower_bound(), solution_pool.get_upper_bound())
    opt_solution = branch_and_bound(instance, cb , initial_solution, 60)

