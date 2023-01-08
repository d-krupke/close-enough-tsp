import typing
from ..core import Instance, TripleMap
import networkx as nx
import itertools

import gurobipy as gp  # requires gurobi to be installed via conda.
from gurobipy import GRB


class TripleVars:
    def __init__(self, graph: nx.Graph, model: gp.Model):
        self.graph = graph
        self._vars = model.addVars(
            self.all_triples(), lb=0, ub=1.0, vtype=GRB.CONTINUOUS
        )
        self.model = model

    def all_triples(self):
        for v in self.graph.nodes:
            for n0, n1 in itertools.combinations(self.graph.neighbors(v), 2):
                yield min(n0, n1), v, max(n0, n1)

    def get(self, n0, v, n1):
        n0, n1 = min(n0, n1), max(n0, n1)
        return self._vars[(n0, v, n1)]

    def var_sum(self, source, target=None):
        if target is None:
            return sum(
                self.get(n0, source, n1)
                for n0, n1 in itertools.combinations(self.graph.neighbors(source), 2)
            )
        return sum(
            self.get(n, source, target)
            for n in self.graph.neighbors(source)
            if n != target
        )

    def get_solution(self, lazy=False):
        graph = nx.Graph()
        for t, x in self._vars.items():
            val = self.model.cbGetSolution(x) if lazy else x.X
            if val > 0.01:
                graph.add_edge(t[0], t[1])
                graph.add_edge(t[1], t[2])
        return graph


class LowerBoundLp:
    def __init__(
        self,
        triple_map: TripleMap,
        sequence: typing.List[int],
        sample: typing.List[int],
    ):
        self.model = gp.Model()
        self.graph = self._create_graph(sequence, sample)
        self.vars = TripleVars(self.graph, self.model)
        self.triple_map = triple_map
        self._enforce_coverage()
        self._enforce_flow()
        self._minimize_costs()

    def _minimize_costs(self):
        obj = sum(
            self.triple_map.get_cost(n0, v, n1) * self.vars.get(n0, v, n1)
            for n0, v, n1 in self.vars.all_triples()
        )
        self.model.setObjective(obj, GRB.MINIMIZE)

    def _enforce_coverage(self):
        for v in self.graph:
            self.model.addConstr(self.vars.var_sum(v) == 1)

    def _enforce_flow(self):
        for v, w in self.graph.edges:
            self.model.addConstr(self.vars.var_sum(v, w) == self.vars.var_sum(w, v))

    def _create_graph(self, sequence, sample):
        graph = nx.Graph()
        # the current tour
        for i, v in enumerate(sequence):
            graph.add_edge(v, sequence[(i + 1) % len(sequence)])

        # connect sample to all other
        for v in sample:
            nodes = list(graph.nodes)
            for n in nodes:
                graph.add_edge(v, n)

        return graph

    def prohibit_subtour(self, subset, lazy=False):
        constr = 0
        for v in subset:
            for n in self.graph.neighbors(v):
                if n in subset:
                    continue
                constr += self.vars.var_sum(v, n)
        if lazy:
            self.model.cbLazy(constr >= 2)
        else:
            self.model.addConstr(constr >= 2)

    def optimize(self):
        def cb(model, where):
            if where == GRB.Callback.MIPSOL:
                g = self.vars.get_solution(lazy=True)
                cc = list(nx.connected_components(g))
                if len(cc) == 1:
                    return
                else:
                    for c in cc:
                        self.prohibit_subtour(c, lazy=True)

        self.model.setParam("OutputFlag", False)
        # self.model.Params.LazyConstraints = 1
        connected = False
        while not connected:
            self.model.optimize()
            g = self.vars.get_solution(lazy=False)
            cc = list(nx.connected_components(g))
            if len(cc) == 1:
                connected = True
                break
            else:
                # print(cc)
                for c in cc:
                    self.prohibit_subtour(c, lazy=False)
        return self.model.objVal
