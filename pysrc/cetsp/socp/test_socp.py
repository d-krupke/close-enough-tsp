from . import compute_tour
from ..common import Circle
import pytest


def test_simple():
    circles = [Circle(0, 0, 1), Circle(3, 0, 1)]
    obj, tour = compute_tour(circles)
    assert obj == pytest.approx(2.0)


def test_simple_path():
    circles = [Circle(0, 0, 1), Circle(3, 0, 1)]
    obj, tour = compute_tour(circles, path=True)
    assert obj == pytest.approx(1.0)
