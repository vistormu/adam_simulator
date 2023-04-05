import mujoco

from ....repository import AdamRepository
from ....entities import AdamInfo


class SimulatedAdamRepository(AdamRepository):
    def init(self, filename: str) -> None:
        # Attributes
        self.model = mujoco.MjModel.from_xml_path(filename)  # type: ignore
        self.data = mujoco.MjData(self.model)  # type: ignore

        # Initial pose
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)  # type: ignore

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)  # type: ignore
