import mujoco

from ....repository import AdamRepository


class SimulatedAdamRepository(AdamRepository):
    def init(self, filename: str) -> None:
        # Attributes
        self.model = mujoco.MjModel.from_xml_path(filename)  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)  # type: ignore

    def close(self) -> None:
        pass
