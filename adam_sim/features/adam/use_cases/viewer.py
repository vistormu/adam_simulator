from mujoco_viewer import MujocoViewer

import time


class Viewer:
    def __init__(self) -> None:
        self.is_active: bool = False
        self.hidden_menu: bool = False

    def init(self, model, data) -> None:
        if not self.is_active:
            self.viewer: MujocoViewer = MujocoViewer(model, data, title='ADAM Simulator')

            self.set_view(center=(0.0, 0.0, 0.5),
                          azimuth=-135.0,
                          elevation=-20.0,
                          distance=3.5,
                          )

            self.is_active = True

    def render(self, hide_menu: bool) -> None:
        if hide_menu and not self.hidden_menu:
            self.viewer._hide_menus = True
            self.hidden_menu = True

        self.viewer.render()

        # TMP
        time.sleep(1.0/60.0)

    def close(self) -> None:
        if self.is_active:
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
