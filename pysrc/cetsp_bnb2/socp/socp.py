import typing

import gurobipy as gp  # requires gurobi to be installed via conda.
from gurobipy import GRB

from ..common import Circle


def compute_tour(
    circle_sequence: typing.List[Circle], path: bool = False, verbose=True
) -> typing.Tuple[float, typing.List[typing.Tuple[float, float]]]:
    """
    This function efficiently computes the optimal CE-Tour for a given sequence(!) of circles.
    If the order of circles is given, the problem becomes an easy to solve SOC-Program.
    The tour may implicitly cross circles earlier.

    Learn more about SOC: https://www.gurobi.com/events/gurobi-qcp-and-socp-optimizer-overview/
    :param circle_sequence: Sequence of circles (the exact order they are visited)
    :param path: Compute a path instead of a tour.
    :return: Tour length and sequence of coordinates of the tour. The coordinates may
            be slightly off (but not critical).
    """
    if verbose:
        print("Building model.")
    model = gp.Model()

    # tour points
    x = model.addVars(
        circle_sequence, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
    )
    y = model.addVars(
        circle_sequence, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
    )

    # length of segments
    f = model.addVars(circle_sequence, lb=0.0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS)
    model.setObjective(gp.quicksum(f.values()), sense=GRB.MINIMIZE)

    # x and y-length of segments (difference of segment points)
    w = model.addVars(
        circle_sequence, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
    )
    u = model.addVars(
        circle_sequence, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
    )

    # difference of tour points to circle center
    s = model.addVars(
        circle_sequence, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
    )
    t = model.addVars(
        circle_sequence, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
    )

    # SOC
    for c in circle_sequence:
        model.addQConstr(f[c] * f[c] >= w[c] * w[c] + u[c] * u[c])
        # tour points are within circle radius
        model.addQConstr(s[c] * s[c] + t[c] * t[c] <= c.radius * c.radius)

        # enforce s,t represent difference to circle center
        model.addConstr(s[c] == c.x - x[c])
        model.addConstr(t[c] == c.y - y[c])

    for i, c in enumerate(circle_sequence):
        if path and i == 0:
            # for paths, the distance between the endpoints
            # is free
            model.addConstr(w[c] == 0)
            model.addConstr(u[c] == 0)
        else:
            # enforce w,u represent segment lengths (x and y axes)
            prev_c = circle_sequence[(i - 1) % len(circle_sequence)]
            model.addConstr(w[c] == x[prev_c] - x[c])
            model.addConstr(u[c] == y[prev_c] - y[c])
    if not verbose:
        model.setParam("OutputFlag", False)
    model.optimize()
    return model.objVal, [(x[c].X, y[c].X) for c in circle_sequence]
