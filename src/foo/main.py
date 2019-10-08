from __future__ import annotations


def to_test() -> bool:
    return True


def add(x: int, y: int) -> int:
    return x + y


class Coordinates:
    x: int
    y: int
    z: int

    def __init__(self, x: int, y: int, z: int):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self) -> str:
        return f"x:{self.x}, y:{self.y}, z:{self.z}"

    def __eq__(self, o: Coordinates) -> bool:
        return self.x == o.x and self.y == o.y and self.z == o.z


class Robot:
    coords: Coordinates

    def __init__(self):
        self.coords = Coordinates(0, 0, 0)

    def set_coords(self, x: int, y: int, z: int):
        self.coords = Coordinates(x, y, z)

    def move_forward(self, n: int):
        self.coords.y += n
