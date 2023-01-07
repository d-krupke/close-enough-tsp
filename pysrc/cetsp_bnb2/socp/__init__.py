"""
This module is responsible to compute the concrete tour for a given
sequence of circles. This looks complicated, but is actually
just a second order cone program.

! There is a faster implementation in `core`, but  it has to be compiled!
"""
# flake8: noqa F401
from .socp import compute_tour
