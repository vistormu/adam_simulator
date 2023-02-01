import mujoco
import mujoco_viewer

from .entities import Configuration, Data

ASSETS_PATH: str = 'src/assets/'


class Simulation:
    def load_scene(self, scene: str) -> None:
        self.model = mujoco.MjModel.from_xml_path(ASSETS_PATH+scene+'.xml')  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # type: ignore

    def send_configuration(self, configuration: Configuration) -> None:
        self.data.qpos[:] = configuration

    def get_data(self) -> Data:
        configuration: Configuration = Configuration(*self.data.qpos)
        data: Data = Data(configuration)
        return data

    def step(self) -> bool:
        mujoco.mj_step(self.model, self.data)  # type: ignore
        return not self.viewer.is_alive

    def render(self) -> None:
        self.viewer.render()

    def close(self) -> None:
        self.viewer.close()
