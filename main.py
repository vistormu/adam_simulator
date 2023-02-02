from adam import Simulation, Configuration, Data, ConfigurationLoader


def main():
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene('scene')

    configuration_list: list[Configuration] = ConfigurationLoader.load('limits(1).csv')

    # configuration_list: list[Configuration] = [Configuration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]*1000

    for configuration in configuration_list:
        sim.render()
        data: Data = sim.step(configuration)

        if not sim.is_alive:
            break

    sim.close()


if __name__ == '__main__':
    main()
