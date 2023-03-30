import mujoco

from ....entities import Configuration


class Controller:
    def init(self, filename: str) -> None:
        self.model = mujoco.MjModel.from_xml_path(filename)  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore

        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # type: ignore

    def set_left_configuration(self, configuration: Configuration) -> None:
        self.data.qpos[:6] = configuration

    def set_right_configuration(self, configuration: Configuration) -> None:
        self.data.qpos[6:] = configuration

    def set_left_velocity(self, velocity: tuple[float, float, float]) -> None:
        raise NotImplementedError()

    def set_right_velocity(self, velocity: tuple[float, float, float]) -> None:
        raise NotImplementedError()

    def set_base_valocity(self, velocity: tuple[float, float, float]) -> None:
        raise NotImplementedError()

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)  # type: ignore

    def get_configuration(self) -> tuple[Configuration, Configuration]:
        return Configuration(*self.data.qpos[:6]), Configuration(*self.data.qpos[6:])

    def get_left_configuration(self) -> Configuration:
        return Configuration(*self.data.qpos[:6])

    def get_right_configuration(self) -> Configuration:
        return Configuration(*self.data.qpos[6:])
