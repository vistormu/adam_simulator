from adam import Simulation
from adam.entities import Configuration, Data


def main():
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene()

    left_configuration: Configuration = initial_data.configuration.left_manipulator
    right_configuration: Configuration = initial_data.configuration.right_manipulator

    while sim.is_alive:
        left_configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)
        right_configuration -= Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)

        sim.render()
        data: Data = sim.step(left_configuration, right_configuration)

    sim.close()


if __name__ == '__main__':
    main()
