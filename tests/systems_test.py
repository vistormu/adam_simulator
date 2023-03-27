import bgplot as bgp

from bgplot.entities import OrientedPoint

from adam import Simulation
from adam.entities import AdamInfo, Configuration


def main():
    sim: Simulation = Simulation()
    initial_info: AdamInfo = sim.load_scene()

    figure: bgp.Graphics = bgp.Graphics()

    figure.set_limits(xlim=(0.0, 0.5), ylim=(0.0, 0.5), zlim=(0.0, 1.5))

    figure.set_view(0, 20)
    figure.disable('grid', 'ticks', 'axes', 'walls')
    figure.set_background_color(bgp.Colors.white)

    while True:
        sim.render(hide_menu=True)
        info: AdamInfo = sim.step(Configuration(1.1912, 0.1833, 0.6938, -0.9294, 3.1415, 0.0), initial_info.right_manipulator.configuration)

        left_systems: list[OrientedPoint] = [OrientedPoint.from_htm(system.to_htm()) for system in info.left_manipulator.systems]
        right_systems: list[OrientedPoint] = [OrientedPoint.from_htm(system.to_htm()) for system in info.right_manipulator.systems]

        figure.add_oriented_points(left_systems, style='.-', length=0.025)
        figure.add_oriented_points(right_systems, style='.-', length=0.025, color=bgp.Colors.red)
        figure.show()


if __name__ == '__main__':
    main()
