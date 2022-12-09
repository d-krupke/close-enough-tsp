import itertools
import typing

from .node import Node
from .partial_solution import PartialSolution
from ..common import TourInstance, Circle
from ..common.tour import Tour

# BnBAlgorithm<RootNodeStrategy, BranchingStrategy, SearchStrategy>
# SolutionPool: Keeps track on the available solutions
# RootNodeStrategy: Gives you the first node for the BnB
# BranchingStrategy: Decides on which circle to branch ad node in the BnB
# SearchStrategy<PruningStrategy>: Decides on which node to continue
# PruningStrategy: Decide if we should explore a node at all

# Callbacks
# on_visiting_node
# on_improved_lower_bound
# on_improved_upper_bound


class NodeQueue:
    def __init__(self):
        self._nodes = []

    def push(self, node: Node, parent=None):
        self._nodes.append(node)

    def pop(self) -> Node:
        return self._nodes.pop()

    def best(self, nodes: typing.Iterable[Node]) -> Node:
        return min(nodes, key=lambda n: n.get_lower_bound())

    def __bool__(self):
        return bool(self._nodes)


class RootNodeStrategy:
    def __init__(self):
        pass

    def __call__(self, instance: TourInstance) -> Node:
        best_tripple = max(
            (PartialSolution(instance, list(seq)) for seq in
             itertools.combinations(instance, 3)),
            key=lambda ps: ps.value())
        return Node(best_tripple)


class BranchingStrategy:
    def __init__(self):
        pass

    def __call__(self, instance: TourInstance, node: Node) -> Circle:
        return max(instance, key=lambda circ: node.partial_solution.distance(circ))


class BnBTree:
    root_node_strategy = RootNodeStrategy()
    branching_strategy = BranchingStrategy()

    def __init__(self, instance: TourInstance, gap=0.01):
        self.gap = gap
        self.instance = instance
        self.root = self.root_node_strategy(instance)
        self.node_queue = NodeQueue()
        self.node_queue.push(self.root)
        self.best_solution = None

    def get_upper_bound(self):
        if self.best_solution is None:
            return float('inf')
        else:
            return self.best_solution.value()

    def add_solution(self, solution: Tour):
        print("Found solution of value", solution.value())
        if solution.value() < self.get_upper_bound():
            self.best_solution = solution

    def step(self) -> bool:
        if not self.node_queue:
            return False
        node = self.node_queue.pop()
        if node.is_feasible():
            print("Node is feasible.")
            self.add_solution(node.partial_solution.tour)
            return True
        if node.get_lower_bound() >= (1 - self.gap) * self.get_upper_bound():
            print(f"Prune {node.get_lower_bound()}")
            return True
        else:
            circ = self.branching_strategy(self.instance, node)
            assert circ not in node.partial_solution
            for branch in node.branch(circ):
                if branch.is_feasible():
                    self.add_solution(branch.partial_solution.tour)
                else:
                    self.node_queue.push(branch)
        return True
