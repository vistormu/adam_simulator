from abc import ABC, abstractmethod

from ....entities import Configuration, System, Velocity, Acceleration
from ..entities import Collision, ManipulatorInfo


class ManipulatorRepository(ABC):
    @abstractmethod
    def init(self, *args, **kwargs) -> None:
        pass

    @abstractmethod
    def set_configuration(self, configuration: Configuration) -> None:
        pass

    @abstractmethod
    def get_configuration(self) -> Configuration:
        pass

    @abstractmethod
    def set_velocity(self, velocity: Velocity) -> None:
        pass

    @abstractmethod
    def get_velocity(self) -> Velocity:
        pass

    @abstractmethod
    def set_acceleration(self, acceleration: Acceleration) -> None:
        pass

    @abstractmethod
    def get_acceleration(self) -> Acceleration:
        pass

    @abstractmethod
    def get_collisions(self) -> Collision:
        pass

    @abstractmethod
    def get_systems(self) -> list[System]:
        pass

    @abstractmethod
    def get_info(self) -> ManipulatorInfo:
        pass

    @abstractmethod
    def close(self) -> None:
        pass
