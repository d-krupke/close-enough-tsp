import typing
import math
from cetsp.common import Circle
import networkx as nx


def dist(p0: Circle, p1: Circle) -> float:
    return math.sqrt((p0.x - p1.x) ** 2 + (p0.y - p1.y) ** 2)


def tsp_lower_bound(instance: typing.List[Circle]):
    # TODO use real TSP solver
    G = create_graph(instance)
    T = nx.minimum_spanning_tree(G)
    weight = T.size(weight="weight")
    weight -= sum(c.radius for c in instance)
    return max(weight, 0)


def tsp_with_partial_order_lower_bound(
    instance: typing.List[Circle], partial_order: typing.List[int]
):
    # If the given order size is <= 3, we can ignore it
    if len(partial_order) <= 3:
        return tsp_lower_bound(instance)

    # 1: Compute a minimum spanning tree T in G
    G = create_graph(instance)
    T = nx.minimum_spanning_tree(G)

    # 2: C := s1 s2 . . . sk s1
    # 3: Let e1 and e2 be the two most expensive edges of C and let C′ := C − e1 − e2
    C1 = []
    for u in partial_order:
        v = partial_order[(u + 1) % len(partial_order)]
        C1.append((u, v, G[u][v]["weight"]))
    C2 = C1.copy()
    e1 = max(C2, key=lambda e: e[2])
    C2.remove(e1)
    e2 = max(C2, key=lambda e: e[2])
    C2.remove(e2)

    # 4: Let P be the path in T connecting the vertices that are incident to e1
    P1 = nx.shortest_path(T, source=e1[0], target=e1[1])
    P1 = [(u, v, G[u][v]["weight"]) for (u, v) in zip(P1, P1[1:])]

    # 5: Compute a minimum perfect matching M on the odd vertices in the multigraph A := T ∪ C′
    A = nx.MultiGraph(T)
    for e in C2:
        G.add_edge(e[0], e[1], weight=e[2])
    A_odd = nx.Graph()
    for u in A.nodes():
        if A.degree[u] % 2 == 1:
            A_odd.add_node(u)
            for v in A_odd.nodes():
                if u != v:
                    A_odd.add_edge(u, v, weight=G[u][v]["weight"])
    M = nx.min_weight_matching(A_odd)
    assert nx.is_perfect_matching(A_odd, M)
    M = [(u, v, G[u][v]["weight"]) for (u, v) in M]

    # 6: Let P′ be the path in A ∪ M \ (C′ ∪ P ) connecting the vertices that are incident to e2
    K = nx.MultiGraph(A)
    K.add_weighted_edges_from(M)
    assert all(d % 2 == 0 for _v, d in K.degree())
    K.remove_edges_from(C2)
    K.remove_edges_from(P1)
    P2 = nx.shortest_path(K, source=e2[0], target=e2[1])
    P2 = [(u, v, G[u][v]["weight"]) for (u, v) in zip(P2, P2[1:])]

    # 7: Starting from the circuit C′ ∪ P ∪ P′, compute an Eulerian tour in A ∪ M that respects the order of t
    K.remove_edges_from(P2)
    assert all(d % 2 == 0 for _v, d in K.degree())
    cc = list(nx.connected_components(K))
    vertex_to_cc = [None] * K.number_of_nodes()
    for c_idx, c in enumerate(cc):
        for u in c:
            assert vertex_to_cc[u] is None
            vertex_to_cc[u] = c_idx

    cc_tours = [list(nx.eulerian_circuit(K.subgraph(c))) for c in cc]
    # Create an initial tour of C′ ∪ P ∪ P′
    tour = []
    for e in C1:
        if e == e1:
            tour += P1
        elif e == e2:
            tour += P2
        else:
            tour.append(e)
    # Construct the full tour by iterating over the initial tour and inserting each individual component's Eulerian tour
    full_tour = []
    is_c_added = [False] * len(cc)
    for e in tour:
        u, v, _d = e
        full_tour.append(e)
        c_idx = vertex_to_cc[v]
        if c_idx is None or is_c_added[c_idx]:
            continue
        # find v in its component Eulerian tour
        c_tour = cc_tours[c_idx]
        v_idx = None
        for w_idx, (w, _) in enumerate(c_tour):
            if w == v:
                v_idx = w_idx
                break
        full_tour += c_tour[v_idx:] + c_tour[:v_idx]
        is_c_added[c_idx] = True

    # 8: Shorten the Eulerian tour to a Hamiltonian tour respecting the order of t.
    visited = [False] * G.number_of_nodes()
    source = full_tour[0][0]
    tour = [source]
    visited[source] = True
    assert source == partial_order[0]
    is_in_partial_order = [False] * G.number_of_nodes()
    for v in partial_order:
        is_in_partial_order[v] = True
    idx_in_partial_order = 1
    for e in full_tour:
        v = e[1]
        if visited[v]:
            continue  # skip visited vertices
        next_in_partial_order = (
            None
            if idx_in_partial_order >= len(partial_order)
            else partial_order[idx_in_partial_order]
        )
        if is_in_partial_order[v] and v != next_in_partial_order:
            continue  # maintain the order, will be visited later
        tour.append(v)
        visited[v] = True
        if is_in_partial_order[v]:
            idx_in_partial_order += 1
    assert all(visited)

    # Calculate the tour weight
    weight = 0
    for i in range(len(tour)):
        u, v = tour[i], tour[(i + 1) % len(tour)]
        weight += G[u][v]["weight"]

    # Calc lower bound
    weight /= 2.5 - 2 / len(partial_order)
    weight -= sum(c.radius for c in instance)
    return max(weight, 0)


def create_graph(instance: typing.List[Circle]):
    G = nx.Graph()
    for u_idx, u in enumerate(instance):
        G.add_node(u_idx)
        for v_idx, v in enumerate(instance[:u_idx]):
            G.add_edge(u_idx, v_idx, weight=dist(u, v))
    return G
