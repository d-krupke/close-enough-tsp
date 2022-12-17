import matplotlib.pyplot as plt


class Circle:
    """
    Instance representation
    """

    def __init__(self, x: float, y: float, radius: float):
        self.x = x
        self.y = y
        self.radius = radius

    def contains(self, x: float, y: float):
        return (x - self.x) ** 2 + (y - self.y**2) <= self.radius * self.radius

    def squared_dist(self, x: float, y: float):
        return (x - self.x) ** 2 + (y - self.y**2)

    def __repr__(self):
        return f"Circle({self.x}, {self.y}, r={self.radius})"

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.radius == other.radius

    def __hash__(self):
        return hash((self.x, self.y, self.radius))


# simple plotting  helper


def plot_circle(ax: plt.Axes, circle: Circle, **kwargs):
    patch = plt.Circle((circle.x, circle.y), radius=circle.radius, **kwargs)
    ax.add_patch(patch)
