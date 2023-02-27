from dataclasses import dataclass

from .configuration_data import ConfigurationData
from .collision_data import CollisionData


@dataclass
class Data:
    '''
    the Data class is a dataclass that contains all the information of the simulation

    Attributes
    ----------
    configuration : ~.entities.ConfigurationData
        the data referring to the configuration of the robot

    collision : ~.entities.CollisionData
        the data referring to the collisions of the robot
    '''
    configuration: ConfigurationData
    collision: CollisionData
