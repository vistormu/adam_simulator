import numpy as np
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

    Methods
    -------
    """
    x: float
    y: float
    z: float

    def __repr__(self) -> str:
        return f'{self.__class__.__name__}({self.x}, {self.y}, {self.z})'

    def __sub__(self, other: 'Point') -> 'Point':
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other: 'Point') -> 'Point':
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, scalar: float) -> 'Point':
        return Point(self.x*scalar, self.y*scalar, self.z*scalar)

    def __rmul__(self, scalar: float) -> 'Point':
        return Point(self.x*scalar, self.y*scalar, self.z*scalar)

    def __truediv__(self, scalar: float) -> 'Point':
        return Point(self.x/scalar, self.y/scalar, self.z/scalar)

    def __neg__(self):
        return Point(-self.x, -self.y, -self.z)

    def __eq__(self, other: 'Point') -> bool:
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __ne__(self, other: 'Point') -> bool:
        return not self.__eq__(other)

    def __round__(self, n: int) -> 'Point':
        return Point(round(self.x, n), round(self.y, n), round(self.z, n))

    def round(self, n: int = 4) -> 'Point':
        '''
        rounds the point to the specified number of decimal places

        Parameters
        ----------
        n : int
            the number of decimal places to round to

        Returns
        -------
        out : ~.entities.Point
            the rounded point
        '''
        return Point(round(self.x, n), round(self.y, n), round(self.z, n))

    def to_numpy(self) -> np.ndarray:
        '''
        converts the point to a numpy array

        Returns
        -------
        out : numpy.ndarray
            the point as a numpy array
        '''
        return np.array(self)

    @classmethod
    def from_numpy(cls, array: np.ndarray) -> 'Point':
        '''
        converts a numpy array to a point

        Parameters
        ----------
        array : numpy.ndarray
            the numpy array to convert

        Returns
        -------
        out : ~.entities.Point
            the point
        '''
        return cls(*array)

    def to_list(self) -> list:
        '''
        converts the point to a list

        Returns
        -------
        out : list
            the point as a list
        '''
        return [self.x, self.y, self.z]

    @classmethod
    def from_list(cls, list: list) -> 'Point':
        '''
        converts a list to a point

        Parameters
        ----------
        list : list
            the list to convert

        Returns
        -------
        out : ~.entities.Point
            the point
        '''
        return cls(*list)
