from adam import Simulation, ConfigurationsManager
from adam.entities import Configuration, AdamInfo


def main():
    sim: Simulation = Simulation()
    initial_info: AdamInfo = sim.load_scene()

    configuration_list: list[Configuration] = ConfigurationsManager.load('test')

    for configuration in configuration_list:
        sim.render()

        info: AdamInfo = sim.step(configuration, initial_info.right_manipulator.configuration)

    sim.close()


if __name__ == '__main__':
    main()
