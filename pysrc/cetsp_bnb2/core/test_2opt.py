# flake8: noqa F401
from ._cetsp_bindings import (
    Point,
    Circle,
    compute_tour_by_2opt,
    Trajectory,
    branch_and_bound,
    Instance,
)


import random


def test_2opt():
    circles = [Circle(Point(x, y), 1) for x in range(10) for y in range(10)]
    instance = Instance(circles)
    traj = compute_tour_by_2opt(instance)
    # traj = compute_tour_by_2opt(circles, False)


def test_bnb():
    circles = [Circle(Point(x, y), 1) for x in range(7) for y in range(7)]
    instance = Instance(circles)
    instance.circles()
    initial_solution = compute_tour_by_2opt(instance)

    def cb(context):
        context.add_solution(initial_solution)
        lb = context.get_lower_bound()
        ub = context.get_upper_bound()
        if lb > 0.95 * ub:
            context.current_node.prune()  # don't evaluate further
        # print("py", node.get_lower_bound(), solution_pool.get_upper_bound())

    opt_solution = branch_and_bound(instance, cb, initial_solution, 60)


def test_bnb2():
    circles = [
        Circle(Point(x * (1 + random.random()), y * (1 + random.random())), 1)
        for x in range(6)
        for y in range(6)
    ]
    instance = Instance(circles, Point(0, 0), Point(10, 10))
    # compute an initial solution via 2opt
    initial_solution = compute_tour_by_2opt(instance)

    def cb(context):
        if context.current_node.is_feasible():
            ub = context.get_upper_bound()
            print(
                f"Found a new solution! The currently best solution has a value of {ub}."
            )

    timelimit = 60
    opt_solution = branch_and_bound(instance, cb, initial_solution, timelimit)
