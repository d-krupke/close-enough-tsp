"""
This file contains a simple entry point to the BnB-algorithm.
You cannot use all advanced features of the BnB-algorithm with this entry point,
but it is easier to use.
"""

from ._cetsp_bindings import (
    Instance,
    Solution,
    branch_and_bound,
    compute_tour_by_2opt,
)
import typing
import logging

_logger = logging.getLogger("CETSP_BnB2")

from ..heuristics import AdaptiveTspHeuristic

__all__ = ["optimize"]


def optimize(
    instance: Instance,
    timelimit: int = 60,
    root_strategy: str = "ConvexHull",
    branching_strategy: str = "FarthestCircle",
    search_strategy: str = "DfsBfs",
    num_threads: int = 8,
    simplify: bool = True,
    rules: typing.Iterable[str] = ("GlobalConvexHullRule",),
    feasibility_tol: float = 0.001,
    optimality_gap: float = 0.01,
    fallback_if_no_concorde: bool = True,
) -> Solution:
    """
    Solves the instance using the BnB-algorithm.
    """
    # compute initial solution
    try:
        heuristic = AdaptiveTspHeuristic(
            [c.center.x for c in instance],
            [c.center.y for c in instance],
            [c.radius for c in instance],
        )
        tour = heuristic.optimize(10)
        initial_solution = Solution(instance, tour, feasibility_tol)
    except RuntimeError as e:
        if fallback_if_no_concorde:
            _logger.warning(
                "Failed to compute initial solution with Concorde (%s). "
                "Falling back to a simple 2-opt heuristic. To prevent this, "
                "set `fallback_if_no_concorde=False`.",
                str(e),
            )
            initial_solution = compute_tour_by_2opt(instance)
        else:
            raise

    # run BnB
    def cb(context):
        pass  # Nothing to do here, but we have to provide a callback

    return branch_and_bound(
        instance=instance,
        callback=cb,
        initial_solution=initial_solution,
        timelimit=timelimit,
        branching_strategy=branching_strategy,
        root_strategy=root_strategy,
        search_strategy=search_strategy,
        num_threads=num_threads,
        simplify=simplify,
        rules=rules,
        feasibility_tol=feasibility_tol,
        optimality_gap=optimality_gap,
    )
