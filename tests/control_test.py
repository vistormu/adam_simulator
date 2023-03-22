from adam import Simulation
from adam.entities import AdamInfo

from vclog import Logger


def main():
    sim: Simulation = Simulation()
    initial_info: AdamInfo = sim.load_scene()

    while True:
        sim.render(hide_menu=True)
        info: AdamInfo = sim.control('left')

        Logger.debug(info.left_manipulator.systems[1], flush=True)


if __name__ == '__main__':
    main()
