"""
This module is responsible to compute the concrete tour for a given
sequence of circles. This looks complicated, but is actually
just a second order cone program.

Potential Optimizations:
 - Adapting the model, so we can add further circles, instead of having
    to recreate it every time.
"""

from .socp import compute_tour
