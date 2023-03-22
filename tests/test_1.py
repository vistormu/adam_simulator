from adam import Simulation
from adam.entities import Configuration, AdamInfo


def main():
    sim: Simulation = Simulation()
    initial_info: AdamInfo = sim.load_scene()

    left_configuration: Configuration = initial_info.left_manipulator.configuration
    right_configuration: Configuration = initial_info.right_manipulator.configuration

    for _ in range(100):
        left_configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)
        right_configuration -= Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)

        sim.render(hide_menu=True)
        info: AdamInfo = sim.step(left_configuration, right_configuration)

    sim.close()


if __name__ == '__main__':
    main()
