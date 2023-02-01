from src.simulation import Simulation
from src.entities import Configuration, Data
from src.core import Logger


def main():
    sim: Simulation = Simulation()
    sim.load_scene('scene')

    configuration: Configuration = sim.get_data().configuration

    done: bool = False
    for step in range(10_000):
        configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.001, 0.0)

        Logger.debug(configuration.to_degrees())

        sim.send_configuration(configuration)

        sim.render()
        done = sim.step()

        if done:
            break

    sim.close()


if __name__ == '__main__':
    main()
