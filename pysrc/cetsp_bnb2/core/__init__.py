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
from ._cetsp_bindings import *

import matplotlib.pyplot as plt


def plot_circle(ax: plt.Axes, circle: Circle, **kwargs):
    patch = plt.Circle(
        (circle.center.x, circle.center.y), radius=circle.radius, **kwargs
    )
    ax.add_patch(patch)


def plot_solution(ax: plt.Axes, instance, trajectory, highlight=None):
    for i, c in enumerate(instance.circles()):
        if highlight and i in highlight:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="green", fill=False)
        elif trajectory and  trajectory.distance(c) <= 0.01*c.radius:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="black", fill=False)
        else:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="red", fill=False)

    tour = [trajectory[i] for i in range(len(trajectory))]
    plt.plot([p.x for p in tour], [p.y for p in tour], "o-")
    ax.set_aspect("equal", "box")
