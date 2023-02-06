import numpy as np

from .collision_detector import CollisionDetector
from ..entities import Data, CollisionData, ConfigurationData, Collision, Configuration


class DataManager:
    def __init__(self) -> None:
        self.configurations: np.ndarray = None  # type:ignore
        self.geometry_1_list: np.ndarray = None  # type:ignore
        self.geometry_2_list: np.ndarray = None  # type:ignore

        self.collision_detector: CollisionDetector = CollisionDetector()

    def update(self, data) -> None:
        self.configurations = data.qpos.copy()
        self.geometry_1_list = data.contact.geom1.copy()
        self.geometry_2_list = data.contact.geom2.copy()

    def get(self) -> Data:
        left_manipulator_configuration: Configuration = Configuration(*self.configurations[0:6])
        right_manipulator_configuration: Configuration = Configuration(*self.configurations[6:12])

        left_manipulator_collision: Collision = self.collision_detector.check_left_manipulator(self.geometry_1_list, self.geometry_2_list)
        right_manipulator_collision: Collision = self.collision_detector.check_right_manipulator(self.geometry_1_list, self.geometry_2_list)

        configuration_data: ConfigurationData = ConfigurationData(left_manipulator_configuration, right_manipulator_configuration)
        collision_data: CollisionData = CollisionData(left_manipulator_collision, right_manipulator_collision)

        return Data(configuration_data, collision_data)
