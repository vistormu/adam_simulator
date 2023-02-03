import mujoco
import mujoco_viewer
import time

from .core import Logger
from .entities import Configuration, Data
from .use_cases import CollisionDetector, DataManager

ASSETS_PATH: str = 'adam/assets/'


class Simulation:
    def __init__(self) -> None:
        self.is_alive: bool = True
        self.data_manager: DataManager = DataManager()

    def load_scene(self, scene: str) -> Data:
        self.model = mujoco.MjModel.from_xml_path(ASSETS_PATH+scene+'.xml')  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore
        self._load_viewer()

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # type: ignore

        self.data_manager.update(self.data)
        return self.data_manager.get()

    def _load_viewer(self) -> None:
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        self.set_view((0, 0, 0.5), 135, -20, 3.5)

    def set_view(self, center: tuple[float, float, float], azimuth: float, elevation: float, distance: float):
        self.viewer.cam.elevation = elevation
        self.viewer.cam.azimuth = azimuth
        self.viewer.cam.distance = distance
        self.viewer.cam.lookat = center

    def step(self, right_configuration: Configuration, left_configuration: Configuration) -> Data:
        # Send configuration
        self.data.qpos[:] = (*left_configuration, *right_configuration)

        # Step on the simulation
        mujoco.mj_step(self.model, self.data)  # type: ignore

        # Get process
        self.is_alive = self.viewer.is_alive

        self.data_manager.update(self.data)
        return self.data_manager.get()

    def render(self, *, fps: int = 15) -> None:
        self.viewer.render()
        time.sleep(1/fps)

    def close(self) -> None:
        self.viewer.close()
