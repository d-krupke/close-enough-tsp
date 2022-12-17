import random
import typing
import datetime
import math

from ..common import Circle
from ..common.tour import Tour
from ..socp import compute_tour


def dist(p0: Circle, p1: Circle) -> float:
    """
    Simple function to compute the distance between two points.
    """
    return math.sqrt((p0.x - p1.x) ** 2 + (p0.y - p1.y) ** 2)


class TimelimitWatcher:
    class Timeout(Exception):
        pass

    def __init__(self, timelimit):
        self.abort_after = datetime.datetime.now() + datetime.timedelta(
            seconds=timelimit
        )

    def is_out_of_time(self):
        return datetime.datetime.now() > self.abort_after

    def __call__(self, *args, **kwargs):
        if self.is_out_of_time():
            raise TimelimitWatcher.Timeout()


class DistMatrix:
    def __init__(self, instance: typing.List[Circle]):
        self._dists = [
            [dist(p0, p1) for p1 in instance] for i, p0 in enumerate(instance)
        ]

    def dist(self, i: int, j: int):
        if i == j:
            return 0.0
        return self._dists[i][j]


class TwoOptOptimizer:
    def __init__(self, instance: typing.List[Circle]):
        self.instance = instance
        self._dist_matrix = DistMatrix(instance)

    def _diff(self, tour, i, j):
        i_ = i - 1 % len(tour)
        j_ = j - 1 % len(tour)
        prev_cost = self._dist_matrix.dist(tour[i_], tour[i]) + self._dist_matrix.dist(
            tour[j_], tour[j]
        )
        changed_cost = self._dist_matrix.dist(
            tour[i_], tour[j_]
        ) + self._dist_matrix.dist(tour[i], tour[j])
        return changed_cost - prev_cost

    def optimization_step(self, tour):
        changed = False
        for j in range(len(tour)):
            for i in range(j):
                if self._diff(tour, i, j) < 0:
                    tour = tour[:i] + tour[i:j][::-1] + tour[j:]
                    changed = True
        return tour, changed

    def optimize(self, tour=None, timelimit=90):
        if not tour:
            tour = list(range(len(self.instance)))
            random.shuffle(tour)
        improved = True
        timelimit_watcher = TimelimitWatcher(timelimit)
        try:
            while improved:
                tour, improved = self.optimization_step(tour)
                timelimit_watcher()
        except TimelimitWatcher.Timeout:
            print("Terminated by timeout.")
        return tour


def find_cetsp_solution(
    circles: typing.List[Circle], repeat: int = 5, verbose=True
) -> Tour:
    two_opt = TwoOptOptimizer(circles)
    best_obj = None
    best_tour = None
    for _ in range(repeat):
        tsp = two_opt.optimize()
        obj, tour = compute_tour([circles[i] for i in tsp], verbose=verbose)
        if best_obj is None or obj < best_obj:
            best_tour = tour
            best_obj = obj
    return Tour(best_tour)
