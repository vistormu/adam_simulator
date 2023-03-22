import numpy as np
from typing import NamedTuple

from .point import Point
from .vector import Vector


class System(NamedTuple):
    """
    a named tuple that contains the information of position and orientation of an oriented point in 3D space

    Attributes
    ----------
    position: ~.entities.point.Point
        the position of the oriented point

    x_axis: ~.entities.vector.Vector
        the x axis of the system

    y_axis: ~.entities.vector.Vector
        the y axis of the system

    z_axis: ~.entities.vector.Vector
        the z axis of the system

    Methods
    -------
    to_htm(self) -> np.ndarray:
        transforms the system to the Homogeneous Transformation Matrix representation

    from_htm(cls, htm) -> System:
        creates an instance of the class from a Homogeneous Transformation Matrix
    """
    position: Point
    x_axis: Vector
    y_axis: Vector
    z_axis: Vector

    def __repr__(self) -> str:
        position: str = f'({self.position.x:.4f}, {self.position.y:.4f}, {self.position.z:.4f})'
        x_axis: str = f'({self.x_axis.u:.4f}, {self.x_axis.v:.4f}, {self.x_axis.w:.4f})'
        y_axis: str = f'({self.y_axis.u:.4f}, {self.y_axis.v:.4f}, {self.y_axis.w:.4f})'
        z_axis: str = f'({self.z_axis.u:.4f}, {self.z_axis.v:.4f}, {self.z_axis.w:.4f})'
        return f'{self.__class__.__name__}(p: {position}, x: {x_axis}, y: {y_axis}, z: {z_axis})'

    def to_htm(self) -> np.ndarray:
        """
        transforms an oriented point instance to the homogeneous transformation matrix representation

        Returns
        -------
        out : np.ndarray
            the homogeneous transformation matrix as a np array
        """
        return np.array([[*self.x_axis, 0.0],
                         [*self.y_axis, 0.0],
                         [*self.z_axis, 0.0],
                         [*self.position, 1.0]]).T

    @classmethod
    def from_htm(cls, htm: np.ndarray):
        '''
        creates an instance of the class from a Homogeneous Transformation Matrix

        Returns
        -------
        out : ~.entities.system.System
            the system
        '''
        position: Point = Point(*htm[0:3, 3])
        x: Vector = Vector(*htm[0:3, 0])
        y: Vector = Vector(*htm[0:3, 1])
        z: Vector = Vector(*htm[0:3, 2])

        return cls(position, x, y, z)
