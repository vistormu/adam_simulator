import numpy as np
import matplotlib.pyplot as plt

from matplotlib.widgets import Slider

from ....entities import Configuration


GUTTER: float = 0.1
MARGIN: float = 0.1
WIDTH: float = 0.03
TOP_SEP: float = 0.3


class ControlVisualizer:
    def __init__(self, title: str) -> None:
        self.title: str = title
        self.is_active: bool = False

    def init(self, initial_configuration: Configuration) -> None:
        if not self.is_active:
            plt.ion()
            self._load_viewer(initial_configuration)
            self._load_values(initial_configuration)

            self.is_active = True

    def _load_viewer(self, initial_configuration: Configuration) -> None:
        self.figure = plt.figure(frameon=False)
        self.figure.suptitle(self.title)

        # Axes
        q1_axes = self.figure.add_axes((MARGIN, 1.0-TOP_SEP, 1.0-MARGIN*2.5, WIDTH))
        q2_axes = self.figure.add_axes((MARGIN,  1.0-TOP_SEP-GUTTER, 1.0-MARGIN*2.5, WIDTH))
        q3_axes = self.figure.add_axes((MARGIN,  1.0-TOP_SEP-GUTTER*2.0, 1.0-MARGIN*2.5, WIDTH))
        q4_axes = self.figure.add_axes((MARGIN,  1.0-TOP_SEP-GUTTER*3.0, 1.0-MARGIN*2.5, WIDTH))
        q5_axes = self.figure.add_axes((MARGIN,  1.0-TOP_SEP-GUTTER*4.0, 1.0-MARGIN*2.5, WIDTH))
        q6_axes = self.figure.add_axes((MARGIN,  1.0-TOP_SEP-GUTTER*5.0, 1.0-MARGIN*2.5, WIDTH))

        # Sliders
        self._q1_slider: Slider = Slider(ax=q1_axes, label=r'$\theta_1$', valmin=-np.pi, valmax=np.pi, valinit=initial_configuration[0], valstep=0.0001, color='#aecdd2', valfmt='%.4f')
        self._q2_slider: Slider = Slider(ax=q2_axes, label=r'$\theta_2$', valmin=-np.pi, valmax=np.pi, valinit=initial_configuration[1], valstep=0.0001, color='#aecdd2', valfmt='%.4f')
        self._q3_slider: Slider = Slider(ax=q3_axes, label=r'$\theta_3$', valmin=-np.pi, valmax=np.pi, valinit=initial_configuration[2], valstep=0.0001, color='#aecdd2', valfmt='%.4f')
        self._q4_slider: Slider = Slider(ax=q4_axes, label=r'$\theta_4$', valmin=-np.pi, valmax=np.pi, valinit=initial_configuration[3], valstep=0.0001, color='#aecdd2', valfmt='%.4f')
        self._q5_slider: Slider = Slider(ax=q5_axes, label=r'$\theta_5$', valmin=-np.pi, valmax=np.pi, valinit=initial_configuration[4], valstep=0.0001, color='#aecdd2', valfmt='%.4f')
        self._q6_slider: Slider = Slider(ax=q6_axes, label=r'$\theta_6$', valmin=-np.pi, valmax=np.pi, valinit=initial_configuration[5], valstep=0.0001, color='#aecdd2', valfmt='%.4f')

    def _load_values(self, initial_configuration: Configuration) -> None:
        self.q1: float = initial_configuration[0]
        self.q2: float = initial_configuration[1]
        self.q3: float = initial_configuration[2]
        self.q4: float = initial_configuration[3]
        self.q5: float = initial_configuration[4]
        self.q6: float = initial_configuration[5]

    def get(self) -> Configuration:
        def update(val):
            self.q1 = self._q1_slider.val
            self.q2 = self._q2_slider.val
            self.q3 = self._q3_slider.val
            self.q4 = self._q4_slider.val
            self.q5 = self._q5_slider.val
            self.q6 = self._q6_slider.val

        self._q1_slider.on_changed(update)
        self._q2_slider.on_changed(update)
        self._q3_slider.on_changed(update)
        self._q4_slider.on_changed(update)
        self._q5_slider.on_changed(update)
        self._q6_slider.on_changed(update)

        return Configuration(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

    def render(self, fps: int) -> None:
        if self.is_active:
            plt.draw()
            plt.pause(1.0/fps)
            self.figure.axes.clear()

    def close(self) -> None:
        if self.is_active:
            plt.close()
