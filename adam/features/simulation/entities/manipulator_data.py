from typing import NamedTuple

from .collision import Collision
from ....entities import Configuration, System


class ManipulatorData(NamedTuple):
    '''
    the Manipulator Data class is a named tuple that contains all the information of the manipulator of the robot

    Attributes
    ----------
    collision : ~.entities.Collision
        the collision data of the manipulator

    configuration : ~.entities.Configuration
        the configuration data of the manipulator

    systems : list[~.entities.System]
        the list of systems of the manipulator

    velocity : float
        the velocity of the manipulator
    '''
    collision: Collision
    configuration: Configuration
    systems: list[System]
    velocity: float
