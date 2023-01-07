"""
Here we implement a basic branch and bound algorithm for CE-TSP.
This version is just for experimental evaluation, the final implementation
has to be in C++ for efficiency (one will have to explore a lot of nodes).

You can find an article on branch and bound here: https://en.wikipedia.org/wiki/Branch_and_bound
"""
# flake8: noqa F401
from .tree import BnBTree
