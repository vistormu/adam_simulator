from ....repository import ManipulatorRepository
from ......entities import Configuration, Vector, System
from ....entities import Collision, ManipulatorInfo


class RealRightManipulatorRepository(ManipulatorRepository):
    def init(self) -> None:
        return super().init()

    def set_configuration(self, configuration: Configuration) -> None:
        return super().set_configuration(configuration)

    def get_configuration(self) -> Configuration:
        return super().get_configuration()

    def set_velocity(self, velocity: Vector) -> None:
        return super().set_velocity(velocity)

    def get_velocity(self) -> Vector:
        return super().get_velocity()

    def get_collisions(self) -> Collision:
        return super().get_collisions()

    def get_systems(self) -> list[System]:
        return super().get_systems()

    def get_info(self) -> ManipulatorInfo:
        return super().get_info()
