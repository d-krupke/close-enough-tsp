"""
This package especially provides the BnB-algorithm including callbacks.
```
circles = [Circle(Point(x, y), 1) for x in range(7) for y in range(7)]
instance = Instance(circles)
instance.circles()
initial_solution = compute_tour_by_2opt(instance)

def cb(context):
    context.add_solution(initial_solution)
    lb = context.get_lower_bound()
    ub = context.get_upper_bound()
    if(lb > 0.95*ub):
        context.current_node.prune()  # don't evaluate further
    print("LB/UB:", contex.get_lower_bound(), context.get_upper_bound())
opt_solution = branch_and_bound(instance, cb , initial_solution, 60)
```
"""
# flake8: noqa F401
from ._cetsp_bindings import Instance, Solution, Circle, Point, branch_and_bound
import typing
import matplotlib.pyplot as plt

from ..heuristics import AdaptiveTspHeuristic

__all__ = ["Instance", "Solution", "Circle", "Point", "optimize", "plot_solution"]


def optimize(
    instance: Instance,
    timelimit: int = 60,
    root_strategy: str = "ConvexHull",
    branching_strategy: str = "FarthestCircle",
    search_strategy: str = "DfsBfs",
    num_threads: int = 8,
    simplify: bool = True,
    rules: typing.Iterable[str] = ("GlobalConvexHullRule",),
    feasibility_tol: float = 0.001,
    optimality_gap: float = 0.01,
) -> Solution:
    """
    Solves the instance using the BnB-algorithm.
    """
    # compute initial solution
    heuristic = AdaptiveTspHeuristic(
        [c.center.x for c in instance],
        [c.center.y for c in instance],
        [c.radius for c in instance],
    )
    tour = heuristic.optimize(10)
    initial_solution = Solution(instance, tour, feasibility_tol)

    # run BnB
    def cb(context):
        pass  # Nothing to do here, but we have to provide a callback

    return branch_and_bound(
        instance=instance,
        callback=cb,
        initial_solution=initial_solution,
        timelimit=timelimit,
        
        branching_strategy=branching_strategy,
        root_strategy=root_strategy,
        search_strategy=search_strategy,
        num_threads=num_threads,
        simplify=simplify,
        rules=rules,
        feasibility_tol=feasibility_tol,
        optimality_gap=optimality_gap,
        
    )


def plot_circle(ax: plt.Axes, circle: Circle, **kwargs):
    patch = plt.Circle(
        (circle.center.x, circle.center.y), radius=circle.radius, **kwargs
    )
    ax.add_patch(patch)


def plot_solution(ax: plt.Axes, instance, solution, highlight=None):
    trajectory = solution.get_trajectory()
    for i, c in enumerate(instance.circles()):
        if highlight and i in highlight:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="green", fill=False)
        elif trajectory and trajectory.distance(c) <= 0.01 * c.radius:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="black", fill=False)
        else:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="red", fill=False)

    tour = [trajectory[i] for i in range(len(trajectory))]
    plt.plot([p.x for p in tour], [p.y for p in tour], "o-")
    ax.set_aspect("equal", "box")
