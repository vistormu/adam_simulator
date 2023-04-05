import numpy as np

from ....repository import ManipulatorRepository
from ......entities import Configuration, Point, Vector, System
from ....entities import Collision, ManipulatorInfo
from .use_cases import CollisionChecker


class SimulatedLeftManipulatorRepository(ManipulatorRepository):
    def init(self, data) -> None:
        self.data = data

        initial_configuration: Configuration = Configuration(1.5708, 0.58905, -0.098, -1.5708, 3.1415, 0.0)
        self.data.qpos[0:6] = initial_configuration
        self.data.ctrl[0:6] = initial_configuration

    def set_configuration(self, configuration: Configuration) -> None:
        self.data.qpos[0:6] = configuration

    def get_configuration(self) -> Configuration:
        return Configuration(*self.data.qpos[0:6])

    def set_velocity(self, velocity: Vector) -> None:
        return super().set_velocity(velocity)

    def get_velocity(self) -> Vector:
        return super().get_velocity()

    def get_collisions(self) -> Collision:
        return CollisionChecker.get(self.data.contact.geom1, self.data.contact.geom2)

    def get_systems(self) -> list[System]:
        systems: list[System] = [System.from_htm(np.array([[*orientation[0:3], position[0]],
                                                           [*orientation[3:6], position[1]],
                                                           [*orientation[6:9], position[2]]]))
                                 for orientation, position in zip(self.data.xmat[2:9], self.data.xpos[2:9])]

        systems[1] = System(systems[1].position, systems[1].x_axis, -systems[1].z_axis, systems[1].y_axis)
        system_2_position: Point = Point(*(systems[1].position + Point(*systems[1].x_axis) * 0.24365 * np.cos(self.data.qpos[1]) + Point(*systems[1].y_axis) * 0.24365 * np.sin(self.data.qpos[1])))
        systems[2] = System(system_2_position, systems[2].z_axis, systems[2].x_axis, systems[2].y_axis)
        system_3_position: Point = Point(*(systems[2].position + Point(*systems[2].x_axis) * 0.21325 * np.cos(self.data.qpos[2]) + Point(*systems[2].y_axis) * 0.21325 * np.sin(self.data.qpos[2])))
        systems[3] = System(system_3_position, systems[3].z_axis, systems[3].x_axis, systems[3].y_axis)
        systems[4] = System(systems[5].position, -systems[4].x_axis, -systems[4].y_axis, systems[4].z_axis)
        systems[5] = System(systems[6].position, -systems[5].x_axis, systems[5].z_axis, systems[5].y_axis)
        system_6_position: Point = Point(*(systems[5].position + Point(*systems[5].z_axis) * 0.0819))
        systems[6] = System(system_6_position, -systems[6].x_axis, systems[6].z_axis, systems[6].y_axis)

        return systems

    def get_info(self) -> ManipulatorInfo:
        collisions: Collision = self.get_collisions()
        configuration: Configuration = self.get_configuration()
        systems: list[System] = self.get_systems()

        return ManipulatorInfo(collisions, configuration, systems, 0.0)
