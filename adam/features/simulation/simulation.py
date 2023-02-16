import mujoco
import mujoco_viewer
import time
import pkg_resources

from .entities import Configuration, Data
from .use_cases import DataManager


class Simulation:
    def __init__(self) -> None:
        self.is_alive: bool = True
        self.data_manager: DataManager = DataManager()

    def load_scene(self, filename: str | None = None) -> Data:
        if filename is None:
            filename = pkg_resources.resource_filename('adam', 'assets/scene.xml')

        self.model = mujoco.MjModel.from_xml_path(filename)  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore
        self._load_viewer()

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # type: ignore

        self.data_manager.update(self.data)
        return self.data_manager.get()

    def _load_viewer(self) -> None:
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        self.set_view((0, 0, 0.5), 135, -20, 3.5)

    def set_view(self, center: tuple[float, float, float], azimuth: float, elevation: float, distance: float) -> None:
        self.viewer.cam.elevation = elevation
        self.viewer.cam.azimuth = azimuth
        self.viewer.cam.distance = distance
        self.viewer.cam.lookat = center

    def extend_collisions(self, collision_dict: dict[int, str]) -> None:
        self.data_manager.collision_detector.collision_dict.update(collision_dict)

    def step(self, left_configuration: Configuration, right_configuration: Configuration) -> Data:
        # Send configuration
        self.data.qpos[:] = (*left_configuration, *right_configuration)

        # Step on the simulation
        mujoco.mj_step(self.model, self.data)  # type: ignore

        # Get process
        self.is_alive = self.viewer.is_alive

        self.data_manager.update(self.data)
        return self.data_manager.get()

    def render(self, *, fps: int = 30) -> None:
        self.viewer.render()
        time.sleep(1/fps)

    def close(self) -> None:
        self.viewer.close()
