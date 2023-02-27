from mujoco_viewer import MujocoViewer
import time


class Viewer:
    def __init__(self) -> None:
        self.is_active: bool = False

    def init(self, model, data) -> None:
        self.viewer: MujocoViewer = MujocoViewer(model, data, title='ADAM Simulator')

        self.set_view(center=(0.0, 0.0, 0.5),
                      azimuth=-135.0,
                      elevation=-20.0,
                      distance=3.5,
                      )

        self.is_active = True

    def render(self, fps: int) -> None:
        self.viewer.render()
        time.sleep(1.0/fps)

    def close(self) -> None:
        self.viewer.close()

    def set_view(self, center: tuple[float, float, float] | None, azimuth: float | None, elevation: float | None, distance: float | None) -> None:
        if center is not None:
            self.viewer.cam.lookat = center

        if azimuth is not None:
            self.viewer.cam.azimuth = azimuth

        if elevation is not None:
            self.viewer.cam.elevation = elevation

        if distance is not None:
            self.viewer.cam.distance = distance
