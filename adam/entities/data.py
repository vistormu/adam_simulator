from dataclasses import dataclass

from .configuration_data import ConfigurationData
from .collision_data import CollisionData


@dataclass
class Data:
    configuration: ConfigurationData
    collision: CollisionData
