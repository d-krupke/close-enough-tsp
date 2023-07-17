import typing

import gurobipy as gp  # requires gurobi to be installed via conda.
from gurobipy import GRB
import random
import math
import logging

_logger = logging.getLogger("CETSP_BnB2")


class AdaptiveTspHeuristic:
    """
    Heuristic for the Close Enough Traveling Salesman Problem (CE-TSP).
    This heuristic iteratively solves the classical TSP on the hitting points.
    The initial hitting points are the centers of the circles, but then
    replaced by the best hitting points via an SOCP.
    This strategy seems to result in near-optimal solutions for the CE-TSP.

    Note that the TSP is still NP-hard, but Concorde can solve it to optimality
    very quickly for the instance sizes we consider.
    """

    def __init__(
        self,
        xs: typing.List[float],
        ys: typing.List[float],
        rs: typing.List[float],
    ) -> None:
        """
        :param xs: x-coordinates of the circle centers
        :param ys: y-coordinates of the circle centers
        :param rs: radii of the circles
        """
        if len(xs) != len(ys) or len(xs) != len(rs):
            raise ValueError("xs, ys, and rs must have the same length")
        self.xs = xs
        self.ys = ys
        self.rs = rs
        self.n = len(xs)
        self.hitting_points_x = list(xs)
        self.hitting_points_y = list(ys)
        self.tour = list(range(len(xs)))
        self.length = float("inf")

    def _get_random_point_in_circle(
        self, x: float, y: float, r: float
    ) -> typing.Tuple[float, float]:
        """Returns a random point in the circle with center
        (x,y) and radius r.
        """
        angle = random.random() * 2 * math.pi
        x_ = x + random.random() * r * math.cos(angle)
        y_ = y + random.random() * r * math.sin(angle)
        return x_, y_

    def randomize_hitting_points(self, ratio=1.0) -> None:
        """
        Randomizes the hitting points by moving them to a random point within the circles.
        The ratio parameter controls how many points are randomized.
        This allows to potentially escape local optima.
        """
        for i in range(self.n):
            if random.random() < ratio:
                (
                    self.hitting_points_x[i],
                    self.hitting_points_y[i],
                ) = self._get_random_point_in_circle(self.xs[i], self.ys[i], self.rs[i])

    def add_circle(self, x: float, y: float, r: float) -> int:
        """
        Add a circle to the instance.
        You have to call optimize() again to recompute the tour.
        """
        self.xs.append(x)
        self.ys.append(y)
        self.rs.append(r)
        self.hitting_points_x.append(x)
        self.hitting_points_y.append(y)
        self.n += 1
        self.tour.append(self.n - 1)
        return len(self.xs) - 1

    _notified_about_failed_concorde_import = False

    def _recompute_tour(self) -> None:
        if len(set((x, y) for x, y in zip(self.xs, self.ys))) <= 2:
            # all circles are on a line, so the tour is trivial
            return
        try:
            from concorde.tsp import TSPSolver

            solver = TSPSolver.from_data(
                # Concorde expects integer coordinates, so we multiply by 1000
                [1000 * x for x in self.hitting_points_x],
                [1000 * y for y in self.hitting_points_y],
                norm="EUC_2D",
            )
            solution = solver.solve(verbose=False)
            self.tour = [int(i) for i in solution.tour]
        except ImportError as e:
            if not self._notified_about_failed_concorde_import:
                self._notified_about_failed_concorde_import = True
                _logger.error(
                    "Failed to import Concorde (%s). Please try to install "
                    "it via `pip install git+https://github.com/jvkersch/pyconcorde`."
                    " Unfortunately, this does not work on all platforms.",
                    str(e),
                )
                raise RuntimeError("No Concorde available.")

    def _recompute_hitting_points(self) -> float:
        model = gp.Model()

        # tour points
        x = model.addVars(
            self.tour, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
        )
        y = model.addVars(
            self.tour, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
        )

        # length of segments
        f = model.addVars(self.tour, lb=0.0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS)
        model.setObjective(gp.quicksum(f.values()), sense=GRB.MINIMIZE)

        # x and y-length of segments (difference of segment points)
        w = model.addVars(
            self.tour, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
        )
        u = model.addVars(
            self.tour, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
        )

        # difference of tour points to circle center
        s = model.addVars(
            self.tour, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
        )
        t = model.addVars(
            self.tour, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS
        )

        # SOC
        for c in self.tour:
            model.addQConstr(f[c] * f[c] >= w[c] * w[c] + u[c] * u[c])
            # tour points are within circle radius
            model.addQConstr(s[c] * s[c] + t[c] * t[c] <= self.rs[c] * self.rs[c])

            # enforce s,t represent difference to circle center
            model.addConstr(s[c] == self.xs[c] - x[c])
            model.addConstr(t[c] == self.ys[c] - y[c])

        for i, c in enumerate(self.tour):
            # enforce w,u represent segment lengths (x and y axes)
            prev_c = self.tour[(i - 1) % len(self.tour)]
            model.addConstr(w[c] == x[prev_c] - x[c])
            model.addConstr(u[c] == y[prev_c] - y[c])

        # no output
        model.setParam("OutputFlag", 0)

        model.optimize()
        self.hitting_points_x = [x[i].x for i in range(self.n)]
        self.hitting_points_y = [y[i].x for i in range(self.n)]
        return model.objVal

    def optimize(self, iterations=10) -> typing.List[int]:
        """ "
        Optimizes the tour.
        :param iterations: number of iterations to run the heuristic
        :return: the tour
        """
        for i in range(iterations):
            self._recompute_tour()
            obj = self._recompute_hitting_points()
            if abs(obj - self.length) < 1e-6:
                break  # no improvement
            self.length = obj
        return list(self.tour)
