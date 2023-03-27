from dataclasses import dataclass

from .point import Point
from .vector import Vector


@dataclass
class BaseData:
    '''
    the Base Data class is a dataclass that contains all the information of the base of the robot

    Attributes
    ----------
    position : ~.entities.Point
        the position of the base of the robot

    velocity : ~.entities.Vector
        the velocity of the base of the robot
    '''
    position: Point
    velocity: Vector
