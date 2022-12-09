import typing
from shapely.geometry.polygon import LinearRing, Point as SPoint

from .instance import TourInstance
from .circle import Circle, plot_circle

Point = typing.Tuple[float, float]


class Tour:
    __EPSILON = 0.001

    def __init__(self, points: typing.List[Point]):
        self.points = points
        if len(points) == 1:
            self._shapely = SPoint(*points[0])
        elif len(points) == 2:
            self._shapely = LinearRing(self.points + self.points[-1:])
        else:
            self._shapely = LinearRing(self.points)

    def value(self) -> float:
        return self._shapely.length

    def __contains__(self, circle: Circle) -> bool:
        return self.distance(circle) <= self.__EPSILON

    def strongly_contains(self, circle: Circle, val: float = 0.025):
        return self.distance(circle) < - circle.radius * val

    def distance(self, circle: Circle) -> float:
        return self._shapely.distance(SPoint(circle.x, circle.y)) - circle.radius


import matplotlib.pyplot as plt


def plot_solution(ax: plt.Axes, instance: TourInstance,
                  solution: Tour):
    for c in instance:
        plot_circle(ax, c, facecolor="white", zorder=1, ec="black")
    tour = solution.points
    plt.plot([p[0] for p in tour] + [tour[0][0]], [p[1] for p in tour] + [tour[0][1]],
             'o-')
    ax.set_aspect('equal', 'box')
