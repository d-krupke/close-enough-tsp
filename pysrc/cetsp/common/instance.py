import typing

from .circle import Circle


class TourInstance:
    def __init__(self, circles: typing.Iterable[Circle]):
        self._circles = list(circles)

    def __getitem__(self, item):
        return self._circles[item]

    def __len__(self):
        return len(self._circles)

    def __iter__(self):
        yield from self._circles
