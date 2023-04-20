from adam_sim import Adam
from adam_sim.entities import Configuration, AdamInfo

from vclog import Logger


def main():
    adam: Adam = Adam()
    info: AdamInfo = adam.connect('localhost', 1883, rate=10)
    # info: AdamInfo = adam.load()

    for _ in range(10):
        Logger.debug(info.left_manipulator.configuration)
        # adam.render()

        left_configuration = info.left_manipulator.configuration + Configuration(0.0, 0.0, 0.0, 0.0, 0.0, 0.1)

        adam.left_manipulator.set_to(left_configuration)

        info = adam.step()

    adam.close()


if __name__ == '__main__':
    main()
