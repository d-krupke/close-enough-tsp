from ._cetsp_bindings import Instance, Solution, Circle, Point
import typing
import matplotlib.pyplot as plt
import logging

_logger = logging.getLogger("CETSP_BnB2")


def plot_circle(ax: plt.Axes, circle: Circle, **kwargs):
    patch = plt.Circle(
        (circle.center.x, circle.center.y), radius=circle.radius, **kwargs
    )
    ax.add_patch(patch)

def plot_instance(ax: plt.Axes, instance, highlight=None):
    for i, c in enumerate(instance.circles()):
        if highlight and i in highlight:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="green", fill=False)
        else:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="red", fill=False)
    
    # make the plot fit the circles
    ax.set_xlim(min([c.center.x - c.radius for c in instance.circles()]) - 1, max([c.center.x + c.radius for c in instance.circles()]) + 1)
    ax.set_ylim(min([c.center.y - c.radius for c in instance.circles()]) - 1, max([c.center.y + c.radius for c in instance.circles()]) + 1)
    ax.set_aspect("equal", "box")


def plot_solution(ax: plt.Axes, instance, solution, highlight=None):
    trajectory = solution.get_trajectory()
    for i, c in enumerate(instance.circles()):
        if highlight and i in highlight:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="green", fill=False)
        elif trajectory and trajectory.distance(c) <= 0.01 * c.radius:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="black", fill=False)
        else:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="red", fill=False)
    ax.set_xlim(min([c.center.x - c.radius for c in instance.circles()]) - 1, max([c.center.x + c.radius for c in instance.circles()]) + 1)
    ax.set_ylim(min([c.center.y - c.radius for c in instance.circles()]) - 1, max([c.center.y + c.radius for c in instance.circles()]) + 1)

    tour = [trajectory[i] for i in range(len(trajectory))]
    plt.plot([p.x for p in tour], [p.y for p in tour], "o-")
    ax.set_aspect("equal", "box")
