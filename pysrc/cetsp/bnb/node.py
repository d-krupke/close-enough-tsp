import typing

from .partial_solution import PartialSolution
from .shared_info import SharedInfo
from ..common import Circle
from ..socp import compute_tour


class Node:
    def __init__(self, partial_solution: PartialSolution, origin=None):
        self.partial_solution = partial_solution
        self.__lb = None
        self.branches = None
        self.origin = origin

    def is_feasible(self) -> bool:
        return self.partial_solution.is_feasible()

    def get_lower_bound(self) -> float:
        if self.__lb is None:
            self.__lb = self.partial_solution.value()
        return self.__lb

    def branch(self, circle: Circle,
               condition: typing.Optional[
                   typing.Callable[[PartialSolution], bool]] = None):
        assert not self.is_feasible()
        print(f"Branch {self} on {circle}.")
        if self.origin is not None and self.origin[
            0].get_lower_bound() < self.get_lower_bound():
            #self.partial_solution.simplify()
            pass
        partial_solutions = [self.partial_solution.insert(i, circle) for i in
                             range(len(self.partial_solution))]
        self.branches = [Node(ps, (self, circle)) for ps in partial_solutions if
                         condition is None or condition(ps)]
        return self.branches

    def __repr__(self):
        return f"Node({self.partial_solution})"
