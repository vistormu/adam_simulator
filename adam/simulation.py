import mujoco
import mujoco_viewer
import time

from .entities import Configuration, Data, Collision

ASSETS_PATH: str = 'adam/assets/'


class Simulation:
    def __init__(self) -> None:
        self.is_alive: bool = True

    def load_scene(self, scene: str) -> Data:
        self.model = mujoco.MjModel.from_xml_path(ASSETS_PATH+scene+'.xml')  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore
        self._load_viewer()

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # type: ignore

        configuration: Configuration = Configuration(*self.data.qpos)
        collision: Collision = Collision(False, False, False, False, False, False)
        data: Data = Data(configuration, collision)

        return data

    def _load_viewer(self) -> None:
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
        self.set_view((0,0,0.5), 135, -20, 3.5)
        
    def set_view(self, center: tuple[float, float, float], azimuth: float, elevation: float, distance: float):
        self.viewer.cam.elevation = elevation
        self.viewer.cam.azimuth = azimuth
        self.viewer.cam.distance = distance
        self.viewer.cam.lookat = center

    def step(self, configuration: Configuration) -> Data:
        # Send configuration
        self.data.qpos[:] = configuration

        # Step on the simulation
        mujoco.mj_step(self.model, self.data)  # type: ignore

        # Get process
        self.is_alive = self.viewer.is_alive

        # Get data
        new_configuration: Configuration = Configuration(*self.data.qpos)
        collision: Collision = Collision(False, False, False, False, False, False)
        data: Data = Data(new_configuration, collision)

        return data

    def render(self, *, fps: int = 15) -> None:
        self.viewer.render()
        time.sleep(1/fps)

    def close(self) -> None:
        self.viewer.close()
