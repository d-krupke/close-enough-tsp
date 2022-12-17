# A modular implementation of a Branch and Bound algorithm for the Close-Enough Traveling Salesman Problem

This implementation shall allow to easily experiment with new lower and upper
bound ideas. If you have no idea of how a branch and bound algorithm works,
check out [this video](https://youtu.be/KMlyhggSqYw).

## Python Interface

You can try out new ideas on the lower bound (biggest issue) using this simple
python interface.

```python
# import the stuff
from cetsp.core import (
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
instance = Instance(circles, Point(0, 0), Point(10, 10))

# compute an initial solution via 2opt
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
- `tests` Your tests for C++ and Python. Read also [this](https://blog.ionelmc.ro/2014/05/25/python-packaging/#the-structure).
- `.clang_format` (C&P) C++-formatting rules, so we have a common standard.
  - Needs to be edited if: You want a different C++-coding style.
- `.flake8` (C&P) Python checking rules
  - Needs to be edited if: The rules do not fit your project. Especially, if there are too many false positives of some rule.
- `.gitignore` (C&P) Automatically ignore system specific files
  - Needs to be edited if: You use some uncommon tool that creates some kind of artifacts not covered by the current rules.
- `pyproject.toml` (C&P) Tells pip the dependencies for running `setup.py`.
  - Needs to be edited if: You use additional/different packages in `setup.py`
- `.pre-commit-config.yaml` (C&P) For applying a set of checks locally. Run, e.g., via `pre-commit run --all-files`.
  - Needs to be edited if: Better tools appear you would like to use, like a better `black` etc.
- `CMakeLists.txt` Defines your C++-project. This is a complex topic we won't dive into here. You should know the basics of CMake to continue.
- `conanfile.txt` Defines the C++-dependencies installed via conan (use CPM within CMakeLists.txt for copy&paste dependencies).
  - Needs to be edited if: You change C++-dependencies.
- `MANIFEST.in` (C&P) Defines all the files that need to be packaged for pip.
  - Needs to be edited if: You need some files included that do not fit the basic coding files, e.g., images.
- `setup.py` Scripts for building and installing the package.
  - Needs to be edited if: You add dependencies, rename the project, want to change metadata, change the project structure, etc.
  - If you don't have any CPP-components yet, you need to set the target to None!
- `requirements.txt` The recommended requirements for development on this package
  - Needs to be edited if: You are using further python packages.
