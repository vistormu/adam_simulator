import mujoco
import mujoco_viewer
import time
import pkg_resources
import numpy as np

from .entities import Configuration, Data
from .use_cases import DataManager


class Simulation:
    def __init__(self) -> None:
        self.is_alive: bool = True
        self.data_manager: DataManager = DataManager()

        self.window: bool = False

    def load_scene(self, filename: str | None = None) -> Data:
        if filename is None:
            filename = pkg_resources.resource_filename('adam', 'assets/scene.xml')

        self.model = mujoco.MjModel.from_xml_path(filename)  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # type: ignore

        self.data_manager.update(self.data)
        return self.data_manager.get()

    def _load_viewer(self) -> None:
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        self.set_view(center=(0.0, 0.0, 0.5),
                      azimuth=-135.0,
                      elevation=-20.0,
                      distance=3.5,
                      )

    def set_view(self, center: tuple[float, float, float] | None, azimuth: float | None, elevation: float | None, distance: float | None) -> None:
        if center is not None:
            self.viewer.cam.lookat = center

        if azimuth is not None:
            self.viewer.cam.azimuth = azimuth

        if elevation is not None:
            self.viewer.cam.elevation = elevation

        if distance is not None:
            self.viewer.cam.distance = distance

    def extend_collisions(self, collision_dict: dict[int, str]) -> None:
        self.data_manager.collision_detector.extend_collisions(collision_dict)

    def check_collisions(self, left_configuration_list: list[Configuration], right_configuration_list: list[Configuration]) -> tuple[np.ndarray, np.ndarray]:
        '''
        returns: self_collision_array, env_collision_array
        '''
        self_collision_array: np.ndarray = np.zeros(len(left_configuration_list))
        env_collision_array: np.ndarray = np.zeros(len(left_configuration_list))
        for i, (left_configuration, right_configuration) in enumerate(zip(left_configuration_list, right_configuration_list)):
            data: Data = self.step(left_configuration, right_configuration)
            if data.collision.left_manipulator.self_collision or data.collision.right_manipulator.self_collision:
                self_collision_array[i] = 1
            if data.collision.left_manipulator.env_collision or data.collision.right_manipulator.env_collision:
                env_collision_array[i] = 1

        return self_collision_array, env_collision_array

    def step(self, left_configuration: Configuration, right_configuration: Configuration) -> Data:
        # Send configuration
        self.data.qpos[:] = (*left_configuration, *right_configuration)

        # Step on the simulation
        mujoco.mj_step(self.model, self.data)  # type: ignore

        # Get process
        # self.is_alive = self.viewer.is_alive

        self.data_manager.update(self.data)
        return self.data_manager.get()

    def render(self, *, fps: int = 30) -> None:
        if not self.window:
            self._load_viewer()
            self.window = True

        self.viewer.render()
        time.sleep(1/fps)

    def close(self) -> None:
        if self.window:
            self.viewer.close()
