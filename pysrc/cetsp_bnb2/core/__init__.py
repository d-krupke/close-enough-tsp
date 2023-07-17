"""
This package especially provides the BnB-algorithm including callbacks.

For simple usage, use ``optimize`` to solve an instance.
It has reasonable default values for the parameters.
For more advanced usage, use ``branch_and_bound`` directly.

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
    print("LB/UB:", context.get_lower_bound(), context.get_upper_bound())
opt_solution = branch_and_bound(instance, cb , initial_solution, 60)
```
"""
# flake8: noqa F401
from ._cetsp_bindings import (
    Instance,
    Solution,
    Trajectory,
    Circle,
    Point,
    branch_and_bound,
    compute_tour_by_2opt,
)
import typing
import matplotlib.pyplot as plt
import logging

_logger = logging.getLogger("CETSP_BnB2")

from ..heuristics import AdaptiveTspHeuristic
from .optimize import optimize
from .plotting import plot_solution, plot_circle

__all__ = [
    "Instance",
    "Solution",
    "Trajectory",
    "Circle",
    "Point",
    "optimize",
    "plot_solution",
]
