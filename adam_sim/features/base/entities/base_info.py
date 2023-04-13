from dataclasses import dataclass

from ....entities import Point, Vector, System


@dataclass
class BaseInfo:
    '''
    the `BaseInfo` class is a dataclass that contains all the information of the base of the robot

    Attributes
    ----------
    position : ~.entities.Point
        the position of the base of the robot

    velocity : ~.entities.Vector
        the velocity of the base of the robot
    '''
    position: Point
    system: System
    velocity: Vector
