from adam_sim import Adam
from adam_sim.entities import AdamInfo

from vclog import Logger


def main():
    adam: Adam = Adam()
    initial_info: AdamInfo = adam.load('tests/assets/scene.xml')

    for _ in range(1000):
        adam.render()
        info: AdamInfo = adam.step()

        Logger.debug(info.left_manipulator.configuration, flush=True)

    adam.close()


if __name__ == '__main__':
    main()
