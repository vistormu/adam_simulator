from dataclasses import dataclass

from .configuration import Configuration
from .collision import Collision


@dataclass
class Data:
    configuration: Configuration
    collision: Collision
