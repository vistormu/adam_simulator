from adam import Simulation, ConfigurationsManager
from adam.entities import Configuration, Data


def main():
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene()

    configuration_list: list[Configuration] = ConfigurationsManager.load('test')

    for configuration in configuration_list:
        sim.render()

        data: Data = sim.step(configuration, initial_data.configuration.right_manipulator)

    sim.close()


if __name__ == '__main__':
    main()
