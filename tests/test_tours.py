import pytest

from cetsp_bnb2 import Circle, Instance, branch_and_bound, Point


def test_empty_tour():
    with pytest.raises(ValueError):
        instance = Instance([])
        solution = branch_and_bound(instance, lambda e: None)


def test_single_circle():
    instance = Instance([Circle(Point(0, 0), 1)])
    solution = branch_and_bound(instance, lambda e: None)


def test_two_circles():
    instance = Instance([Circle(Point(0, 0), 1), Circle(Point(10, 0), 1)])
    ub, lb = branch_and_bound(instance, lambda e: None)
    assert ub.get_trajectory().length() == pytest.approx(16)
    assert lb == pytest.approx(16)


def test_three_circle():
    instance = Instance(
        [Circle(Point(0, 0), 1), Circle(Point(10, 0), 1), Circle(Point(5, 0), 0.0)]
    )
    ub, lb = branch_and_bound(instance, lambda e: None)
    assert ub.get_trajectory().length() == pytest.approx(16)
    assert lb == pytest.approx(16)


def test_square():
    instance = Instance(
        [
            Circle(Point(0, 0), 0),
            Circle(Point(10, 0), 0),
            Circle(Point(0, 10), 0.0),
            Circle(Point(10, 10), 0),
        ]
    )
    ub, lb = branch_and_bound(instance, lambda e: None)
    assert ub.get_trajectory().length() == pytest.approx(40)
    assert lb == pytest.approx(40)


def test_square_with_middle():
    instance = Instance(
        [
            Circle(Point(0, 0), 0),
            Circle(Point(10, 0), 0),
            Circle(Point(0, 10), 0.0),
            Circle(Point(10, 10), 0),
            Circle(Point(5, 5), 0),
        ]
    )
    ub, lb = branch_and_bound(instance, lambda e: None)
    assert ub.get_trajectory().length() == pytest.approx(44.14213093474119)
    assert lb == pytest.approx(44.14213093474119)


def test_square_with_square_middle():
    instance = Instance(
        [
            Circle(Point(0, 0), 0),
            Circle(Point(10, 0), 0),
            Circle(Point(0, 10), 0.0),
            Circle(Point(10, 10), 0),
            Circle(Point(5.5, 5.5), 0),
            Circle(Point(4.5, 4.5), 0),
            Circle(Point(4.5, 5.5), 0),
            Circle(Point(5.5, 4.5), 0),
        ]
    )
    ub, lb = branch_and_bound(instance, lambda e: None)
    assert ub.get_trajectory().length() == pytest.approx(45.7279208391827)
    assert lb <= 45.7279208391827


def test_4x4():
    instance = Instance(
        [Circle(Point(x, y), 0) for x in range(0, 4) for y in range(0, 4)]
    )
    ub, lb = branch_and_bound(instance, lambda e: None)
    assert ub.get_trajectory().length() == pytest.approx(16)
    assert lb <= 16


def test_4x5():
    instance = Instance(
        [Circle(Point(x, y), 0) for x in range(0, 4) for y in range(0, 5)]
    )
    ub, lb = branch_and_bound(instance, lambda e: None)
    assert ub.get_trajectory().length() == pytest.approx(20, rel=0.001)
    assert lb <= 20