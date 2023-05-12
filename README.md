# (Work in Progress) CE-TSP BnB2: An exact and modular Close-Enough Traveling Salesman Problem Solver

*Developed by [Dominik Krupke](https://www.ibr.cs.tu-bs.de/users/krupke/) and Barak Ugav at Tel Aviv University. Further development at TU Braunschweig with Michael Perk.*

The *Close-Enough Traveling Salesman Problem* asks for the shortest tour that visits a given set of circles.
It is related to the classical [Traveling Salesman Problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem)
but more challenging as the distances between two successive cities are not constant.
The problem is NP-hard, and thus, very challenging to solve to optimality.
We provide an algorithm implementation that is capable of solving reasonably sized instances to provable optimality
within seconds to minutes.
Depending on the character of the instance, the solvable instance size is between 20 to multiple hundred circles.

The implementation is a [Branch and Bound algorithm](https://youtu.be/KMlyhggSqYw) making use of a Second Order Cone Program, building on
the work of [Coutinho et al.](https://optimization-online.org/2014/02/4248/).

Primary differences are:
- A highly modular implementation and availability of many search and branching strategies.
- Warm starts with initial solutions provided by heuristics.
- Callbacks allowing to lazy constraints, custom heuristics, and generally heavily influencing the search.
- Parallelization of the search.
- New pruning rules based on geometric insights that can give significant speed-ups.
- Branching degree reduction giving further exponential speed-ups in some cases.

*Cite as "CE-TSP BnB2: An exact and modular Close-Enough Traveling Salesman Problem Solver" by D. Krupke, M. Perk, B. Ugav.*

## Installation

You need a properly installed Gurobi-license for this modul, as we use its SOCP-solver.
You can easily get a free license for academic purposes.
The free non-academic license is probably not sufficient and will lead to errors.

(CGAL does not require you to install a license, but you would need to buy one for commercial usage)

### Python

You can just check out this repository and run

```shell
pip install .
```

Afterward, test the installation with

```shell
pytest -s tests
```

### C++

* Copy code into subfolder.
* Install conan dependencies.
* Add subfolder via `add_subdirectory` to your CMakeList.txt

#### Conan

TODO: Write a conan setup file to allow easy installation via conan.

## Usage

### Python

A simple usage could look like this

```python
from cetsp_bnb2 import Circle, Instance, branch_and_bound, Point

# create some instance
instance = Instance(
    [Circle(Point(x, y), 0) for x in range(0, 4) for y in range(0, 5)]
)

# solve instance
tour, lb, stats = branch_and_bound(instance, lambda e: None)

# access and plot solution
import matplotlib.pyplot as plt

def plot_circle(ax: plt.Axes, circle: Circle, **kwargs):
    patch = plt.Circle(
        (circle.center.x, circle.center.y), radius=circle.radius, **kwargs
    )
    ax.add_patch(patch)


def plot_solution(ax: plt.Axes, instance, solution, highlight=None):
    trajectory = solution.get_trajectory()
    for i, c in enumerate(instance.circles()):
        if highlight and i in highlight:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="green", fill=False)
        elif trajectory and trajectory.distance(c) <= 0.01 * c.radius:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="black", fill=False)
        else:
            plot_circle(ax, c, facecolor="white", zorder=1, ec="red", fill=False)

    tour = [trajectory[i] for i in range(len(trajectory))]
    plt.plot([p.x for p in tour], [p.y for p in tour], "o-")
    ax.set_aspect("equal", "box")

plt.figure()
plot_solution(plt.gca(), instance, tour)
```

### C++

You can also access the solver directly via C++.

TBD


## Related Work

This Branch and Bound algorithm is an improved version of [this paper](https://optimization-online.org/2014/02/4248/).

This [master thesis](https://dspace.cvut.cz/bitstream/handle/10467/96747/F3-DP-2021-Fanta-Lukas-Fanta_Lukas_The_Close_Enough%20_Travelling%20_Salesman_Problem_in_polygonal_domain.pdf?sequence=-1&isAllowed=y)
provides a good overview over the previous work and provides an LNS-algorithm
(highly effective and modern heuristic, probably the most important heuristic
out there for hard problems, even if barely teached).

This [paper](https://www.researchgate.net/profile/Carmine-Cerrone/publication/308037271_A_Novel_Discretization_Scheme_for_the_Close_Enough_Traveling_Salesman_Problem/links/5af8721da6fdcc0c03289b38/A-Novel-Discretization-Scheme-for-the-Close-Enough-Traveling-Salesman-Problem.pdf)
uses a GTSP-MIP for solving the CE-TSP.
The reduction techniques and geometric observations may be intersting for us.

## Modules

[include/cetsp/bnb.h](include/cetsp/bnb.h)

The branch and bound algorithm can be easily modified by replacing strategies.
There are three primary strategies:

### Root Node Strategy

[include/cetsp/details/root_node_strategy.h](include/cetsp/strategies/root_node_strategy.h)

The root node strategy decides the initial relaxed solution.
It should allow to build an optimal solution. We currently have two different
ones

- _LongestEdgeFarthestCircle_: The classical idea also used in previous work that starts with a longest edge and
  additionally the farthest circle. Thus, the initial tour is a triangle. For paths it is just the farthest circle to
  source and target of the path.
- _ConvexHull_: Start with the simplified convex hull. This is needed to do the convex hull pruning.

### Branching Strategies

[include/cetsp/strategies/branching_strategy.h](include/cetsp/strategies/branching_strategy.h)

The branching strategy decides how to split the solution space of a node further.
For classical MIP-solver, this usually requires to select a fractional variable
and rounding it up or down, essentially splitting the solution space into two
parts. For our problem, the branching is easier on selecting what to branch (
try to include the farthest circle in some position as this drives up to
lower bound the most), but the split is much larger as we have to consider
inserting a missing circle anywhere in the sequence. A lot of the resulting
sequences allow arguments why they cannot lead to an optimal solution, and thus
can be pruned, without doing the costly trajectory computation. We allow the
addition of `rules` to improve the performance of the branching.

The branching also triggers the computation of the newly generated children
in parallel to utilize the CPU better. This is not the perfect place for
parallelization but the easiest, as it can easily be synchronized while still
having big impact for the more complicated instances.

A further point here is simplification: We can shorten the sequence and thus
the potential number of branches by only keeping the spanning circles in the
sequence. Removing implicitly covered circles does not lower the costs.

We have the following branching strategies:

- _FarthestCircle_: Just tries to branch on the farthest circle. Allows the addition of further rules.
- _ChFarthestCircle_: Extension of the previous strategy by a rule that makes sure each sequence follows the convex hull, which can be proved to be optimal.
- _RandomCircle_: Just tries to branch on a random (uncovered) circle.

Further ideas:

- Add a rule that directly excludes all sequences that can be lower bounded above the current best solution, without computing the trajectory. We can take the parent's trajectory, remove the maximal cost of the edge on which we want to add a further circle, and then add the minimal cost of traveling to this circle. If it is already higher, we do not have to compute its trajectory.
- The convex hull idea can probably be extended recursively.

#### Sequence Rules

[include/cetsp/strategies/rule.h](include/cetsp/strategies/rule.h)

You can easily define a new rule to directly exclude any suboptimal branches,
if you can find an argument that does not require computing the trajectory.
If you need the trajectory, use a callback and then prune in it. This way,
the trajectory will be cached.

### Search Strategy

[include/cetsp/details/search_strategy.h](include/cetsp/strategies/search_strategy.h)

Which leaf of the branch and bound tree should be investigated next?

- _Depth First_: This strategy always goes into the best child of the current node. This helps to find good feasible
  solutions quickly.
- _Breadth First_: Always uses the cheapest leaf, which improves the lower bound but will take a long time to find a
  feasible solution.
- _Mixed_: Will do depth first until the node gets pruned or becomes feasible. Then it will look for the cheapest leaf.
  This is a popular technique used by modern MILP-solvers.

### Callbacks

TBD

## New Ideas

1. The convex hull enforces quite a lot of order into partial solutions. This yields quite some speed ups for important
   instances in our context.
2. Improved Branching of switching between depth (going deeper until feasible) and breadth first (continue on cheapest
   partial solution).
3. Simplifying branch sequences by removing hitting points that do not span the trajectory. This reduces the exponential
   branching factor, but can also make it more difficult to find feasible solutions.
4. Allowing lazy constraints, which are important for the use as coverage path plannning routine.

## Open Ideas

1. Using intersections as argument for higher lower bounds: We know that any optimal solution will have no intersections
   so the intersections of partial solutions have to be resolved be routinng the tour around one of the components,
   which often is expensive. **This idea is not trivial to implement**
2. Using implicit coverages as argument for lower bounds: If a circle is covered implicitly, it should not have a
   spanning point in the optimal solution. Maybe one can also use this to improve the lower bound, as in order to become
   an optimal solution, the circle either cannot be implicitly covered or its hitting point has to become non-spanning.
   **again complicated geometry that has to be argued for correctness.**
3. Interleaving the search process with heuristics as done in commercial BnB-solvers.
4. Adapting the ideas for optimizing close-enough paths, which may be an even more important subroutine and not as much
   analyzed by others.

## Python Interface

You can try out new ideas on the lower bound (biggest issue) using this simple
python interface.

```python
# import the stuff
from cetsp_bnb2 import (
    Circle,
    Instance,
    compute_tour_by_2opt,
    branch_and_bound,
    Point,
    plot_solution,
)

import random

circles = [
    Circle(Point(x * (1 + random.random()), y * (1 + random.random())), 1)
    for x in range(7)
    for y in range(7)
]
instance = Instance(circles)

# trigger_lazy_computation an initial solution via 2opt
initial_solution = compute_tour_by_2opt(instance)


def cb(context):
    relaxed_sol = context.get_relaxed_solution()
    for c in circles:
        dist = relaxed_sol.distance(c)
        if dist > 0:
            # this may not really be a lower bound.
            context.current_node.add_lower_bound(relaxed_sol.length() + dist)


timelimit = 60
opt_solution = branch_and_bound(instance, cb, initial_solution, timelimit)

# plot
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 10))
plot_solution(plt.gca(), instance, opt_solution)
plt.show()
```

## Project Structure

Please follow the following project structure. It is carefully designed and follows
common guidelines. It uses
[Scikit-HEP developer information](https://scikit-hep.org/developer/intro) as baseline.

- `cmake` Should contain all cmake utilities (no cmake package manager, so copy&paste)
- `include` Public interfaces of C++-libraries you write. Should follow `include/libname/header.h`
- `pysrc` Python packages you write. Should be of the shape `pysrc/package_name/module_name/file.py`. Can be recursive.
- `src` Your implementation internals. Can also contain header files that are not part of the public interface!
- `tests` Your tests for C++ and Python. Read
  also [this](https://blog.ionelmc.ro/2014/05/25/python-packaging/#the-structure).
- `.clang_format` (C&P) C++-formatting rules, so we have a common standard.
  - Needs to be edited if: You want a different C++-coding style.
- `.flake8` (C&P) Python checking rules
  - Needs to be edited if: The rules do not fit your project. Especially, if there are too many false positives of
    some rule.
- `.gitignore` (C&P) Automatically ignore system specific files
  - Needs to be edited if: You use some uncommon tool that creates some kind of artifacts not covered by the current
    rules.
- `pyproject.toml` (C&P) Tells pip the dependencies for running `setup.py`.
  - Needs to be edited if: You use additional/different packages in `setup.py`
- `.pre-commit-config.yaml` (C&P) For applying a set of checks locally. Run, e.g., via `pre-commit run --all-files`.
  - Needs to be edited if: Better tools appear you would like to use, like a better `black` etc.
- `CMakeLists.txt` Defines your C++-project. This is a complex topic we won't dive into here. You should know the basics
  of CMake to continue.
- `conanfile.txt` Defines the C++-dependencies installed via conan (use CPM within CMakeLists.txt for copy&paste
  dependencies).
  - Needs to be edited if: You change C++-dependencies.
- `MANIFEST.in` (C&P) Defines all the files that need to be packaged for pip.
  - Needs to be edited if: You need some files included that do not fit the basic coding files, e.g., images.
- `setup.py` Scripts for building and installing the package.
  - Needs to be edited if: You add dependencies, rename the project, want to change metadata, change the project
    structure, etc.
  - If you don't have any CPP-components yet, you need to set the target to None!
- `requirements.txt` The recommended requirements for development on this package
  - Needs to be edited if: You are using further python packages.
