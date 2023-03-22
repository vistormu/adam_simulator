import numpy as np

from .collision_detector import CollisionDetector
from ..entities import AdamInfo, ManipulatorData, Collision, Configuration, System


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
        left_systems: list[System] = [System.from_htm(np.array([[*orientation[0:3], position[0]],
                                                                [*orientation[3:6], position[1]],
                                                                [*orientation[6:9], position[2]]]))
                                      for orientation, position in zip(data.xmat[2:9], data.xpos[2:9])]

        right_systems: list[System] = [System.from_htm(np.array([[*orientation[0:3], position[0]],
                                                                [*orientation[3:6], position[1]],
                                                                 [*orientation[6:9], position[2]]]))
                                       for orientation, position in zip(data.xmat[9:], data.xpos[9:])]

        left_manipulator_data: ManipulatorData = ManipulatorData(left_collision, left_configuration, left_systems)
        right_manipulator_data: ManipulatorData = ManipulatorData(right_collision, right_configuration, right_systems)

        return AdamInfo(left_manipulator_data, right_manipulator_data)
