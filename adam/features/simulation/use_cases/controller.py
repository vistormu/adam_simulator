import mujoco

from ..entities import Configuration


class Controller:
    def init(self, filename: str) -> None:
        self.model = mujoco.MjModel.from_xml_path(filename)  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # type: ignore

    def set_configuration(self, left_configuration: Configuration, right_configuration: Configuration) -> None:
        self.data.qpos[:] = (*left_configuration, *right_configuration)

    def set_velocity(self, velocity) -> None:
        raise NotImplementedError()

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)  # type: ignore

    def get_configuration(self) -> tuple[Configuration, Configuration]:
        return Configuration(*self.data.qpos[:6]), Configuration(*self.data.qpos[6:])

    def get_left_configuration(self) -> Configuration:
        return Configuration(*self.data.qpos[:6])

    def get_right_configuration(self) -> Configuration:
        return Configuration(*self.data.qpos[6:])
