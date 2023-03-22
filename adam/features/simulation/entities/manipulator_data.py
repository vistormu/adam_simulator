from typing import NamedTuple

from .collision import Collision
from ....core.entities.configuration import Configuration
from .system import System


class ManipulatorData(NamedTuple):
    collision: Collision
    configuration: Configuration
    systems: list[System]
