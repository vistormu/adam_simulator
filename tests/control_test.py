from adam import Simulation
from adam.entities import Data


def main():
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene()

    while True:
        sim.render()
        data: Data = sim.control('right')

    sim.close()


if __name__ == '__main__':
    main()
