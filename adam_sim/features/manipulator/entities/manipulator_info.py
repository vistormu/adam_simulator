from typing import NamedTuple

from .collision import Collision
from ....entities import Configuration, System, Velocity, Acceleration


class ManipulatorInfo(NamedTuple):
    '''
    the `ManipulatorInfo` class is a named tuple that contains all the information of the manipulator of the robot

    Attributes
    ----------
    systems : list[~.entities.System]
        the list of systems of the manipulator

    end_effector : ~.entities.System
        the end effector of the manipulator

    configuration : ~.entities.Configuration
        the configuration data of the manipulator

    velocity : ~.entities.Velocity
        the velocity of the manipulator

    acceleration : ~.entities.Acceleration
        the acceleration of the manipulator

    collision : ~.entities.Collision
        the collision data of the manipulator
    '''
    systems: list[System]
    end_effector: System
    configuration: Configuration
    velocity: Velocity
    acceleration: Acceleration
    collision: Collision
