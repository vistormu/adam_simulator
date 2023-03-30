import numpy as np

from .collision_detector import CollisionDetector
from ..entities import AdamInfo, ManipulatorData, Collision, BaseData
from ....entities import Configuration, System, Point, Vector


def _get_left_systems(data) -> list[System]:
    systems: list[System] = [System.from_htm(np.array([[*orientation[0:3], position[0]],
                                                       [*orientation[3:6], position[1]],
                                                       [*orientation[6:9], position[2]]]))
                             for orientation, position in zip(data.xmat[2:9], data.xpos[2:9])]

    systems[1] = System(systems[1].position, systems[1].x_axis, -systems[1].z_axis, systems[1].y_axis)
    systems[2] = System(systems[3].position, systems[2].z_axis, systems[2].x_axis, systems[2].y_axis)
    systems[3] = System(systems[4].position, systems[3].z_axis, systems[3].x_axis, systems[3].y_axis)
    systems[4] = systems[5]
    systems[5] = System(systems[6].position, -systems[6].x_axis, systems[6].z_axis, systems[6].y_axis)

    step_matrix: np.ndarray = np.array([[np.cos(data.qpos[5]), -np.sin(data.qpos[5]), 0.0, 0.0],
                                        [np.sin(data.qpos[5]), np.cos(data.qpos[5]), 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0819],
                                        [0.0, 0.0, 0.0, 1.0]])

    systems[6] = System.from_htm(systems[5].to_htm() @ step_matrix)

    return systems


def _get_right_systems(data) -> list[System]:
    systems: list[System] = [System.from_htm(np.array([[*orientation[0:3], position[0]],
                                                       [*orientation[3:6], position[1]],
                                                       [*orientation[6:9], position[2]]]))
                             for orientation, position in zip(data.xmat[9:], data.xpos[9:])]

    systems[1] = System(systems[1].position, systems[1].x_axis, -systems[1].z_axis, systems[1].y_axis)
    systems[2] = System(systems[3].position, systems[2].z_axis, systems[2].x_axis, systems[2].y_axis)
    systems[3] = System(systems[4].position, systems[3].z_axis, systems[3].x_axis, systems[3].y_axis)
    systems[4] = systems[5]
    systems[5] = System(systems[6].position, -systems[6].x_axis, systems[6].z_axis, systems[6].y_axis)

    step_matrix: np.ndarray = np.array([[np.cos(data.qpos[5]), -np.sin(data.qpos[5]), 0.0, 0.0],
                                        [np.sin(data.qpos[5]), np.cos(data.qpos[5]), 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0819],
                                        [0.0, 0.0, 0.0, 1.0]])

    systems[6] = System.from_htm(systems[5].to_htm() @ step_matrix)

    return systems


class DataManager:
    def __init__(self) -> None:
        self.collision_detector: CollisionDetector = CollisionDetector()

    def get(self, data) -> AdamInfo:
        # Configurations
        left_configuration: Configuration = Configuration(*data.qpos[0:6])
        right_configuration: Configuration = Configuration(*data.qpos[6:12])

        # Collisions
        left_collision: Collision = self.collision_detector.check_left_manipulator(data.contact.geom1, data.contact.geom2)
        right_collision: Collision = self.collision_detector.check_right_manipulator(data.contact.geom1, data.contact.geom2)

        # Systems
        left_systems: list[System] = _get_left_systems(data)
        right_systems: list[System] = _get_right_systems(data)

        # Velocity
        left_velocity: float = 0.0
        right_velocity: float = 0.0

        left_manipulator_data: ManipulatorData = ManipulatorData(left_collision, left_configuration, left_systems, left_velocity)
        right_manipulator_data: ManipulatorData = ManipulatorData(right_collision, right_configuration, right_systems, right_velocity)

        # Base
        base_data: BaseData = BaseData(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0))

        return AdamInfo(left_manipulator_data, right_manipulator_data, base_data)
