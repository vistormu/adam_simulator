from typing import NamedTuple


class Point(NamedTuple):
    """
    a named tuple that contains the information of a 3D point in space

    Attributes
    ----------
    x : float
        the x value of the point
    y : float
        the y value of the point
    z : float
        the z value of the point
    """
    x: float
    y: float
    z: float

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}({self.x}, {self.y}, {self.z})'

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, scalar):
        return Point(self.x*scalar, self.y*scalar, self.z*scalar)

    def __rmul__(self, scalar):
        return Point(self.x*scalar, self.y*scalar, self.z*scalar)

    def __truediv__(self, scalar):
        return Point(self.x/scalar, self.y/scalar, self.z/scalar)
