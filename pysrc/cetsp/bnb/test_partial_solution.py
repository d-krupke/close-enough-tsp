from .partial_solution import PartialSolution
from ..common import TourInstance, Circle


def test_trivial_feasible():
    instance = TourInstance([Circle(0, 0, 1), Circle(1.5, 0.5, 1), Circle(3, 0, 1)])
    partial_solution = PartialSolution(
        instance, [Circle(0, 0, 1), Circle(1.5, 0.5, 1), Circle(3, 0, 1)]
    )
    assert partial_solution.is_feasible()
    assert Circle(1.5, 0.5, 1) in partial_solution
    assert Circle(1.5, 3, 1) not in partial_solution


def test_other():
    instance = TourInstance(
        [
            Circle(0, 0, 1),
            Circle(3, 0, 1),
            Circle(5, 2, 1),
            Circle(3, 3, 1),
            Circle(0, 4, 1),
        ]
    )
    part_sol = PartialSolution(
        instance,
        [
            Circle(0, 0, 1),
            Circle(5, 2, 1),
            Circle(3, 3, 1),
            Circle(3, 0, 1),
            Circle(0, 4, 1),
        ],
    )
    assert Circle(0, 4, 1) in part_sol
    assert part_sol.is_feasible()
