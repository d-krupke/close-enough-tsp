import typing

from ..common import Circle

from ..common.instance import TourInstance
from ..common.tour import Tour
from ..socp import compute_tour


class PartialSolution:

    def __init__(self, instance: TourInstance,
                 circle_sequence: typing.List[Circle]):
        self.circle_sequence = circle_sequence
        value, points = compute_tour(circle_sequence, verbose=False)
        self.tour = Tour(points)
        self.instance = instance
        self.__is_feasible = None

    def __len__(self):
        return len(self.circle_sequence)

    def insert(self, index: int, circle: Circle):
        return PartialSolution(self.instance, self.circle_sequence[:index] + [
            circle] + self.circle_sequence[index:])

    def __contains__(self, circle: Circle) -> bool:
        return circle in self.tour

    def distance(self, circle: Circle) -> float:
        return self.tour.distance(circle)

    def value(self) -> float:
        return self.tour.value()

    def is_feasible(self) -> bool:
        if self.__is_feasible is None:
            self.__is_feasible = all(circ in self for circ in self.instance)
        return self.__is_feasible

    def simplify(self):
        simplified = [circ for circ in self.circle_sequence if
                                not self.tour.strongly_contains(circ)]
        if len(simplified)<len(self.circle_sequence):
            print(f"Simplified to {len(simplified)} from {len(self.circle_sequence)} ")
        self.circle_sequence = simplified

    def __repr__(self):
        return f"PartialSolution({self.circle_sequence})"
