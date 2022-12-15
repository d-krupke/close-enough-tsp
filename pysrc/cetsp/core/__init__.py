from ._cetsp_bindings import *

import matplotlib.pyplot as plt

def plot_circle(ax: plt.Axes, circle: Circle, **kwargs):
    patch = plt.Circle((circle.center.x, circle.center.y), radius=circle.radius, **kwargs)
    ax.add_patch(patch)

def plot_solution(ax: plt.Axes, instance, trajectory):
    for c in instance.circles():
        plot_circle(ax, c, facecolor="white", zorder=1, ec="black")

    tour = [trajectory[i] for i in range(len(trajectory))]
    plt.plot([p.x for p in tour], [p.y for p in tour], 'o-')
    ax.set_aspect('equal', 'box')