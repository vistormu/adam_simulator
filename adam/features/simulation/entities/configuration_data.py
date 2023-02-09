from dataclasses import dataclass

from ....core.entities.configuration import Configuration


@dataclass
class ConfigurationData:
    left_manipulator: Configuration
    right_manipulator: Configuration
    