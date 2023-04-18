from adam_sim import Adam
from adam_sim.entities import Configuration, AdamInfo

from vclog import Logger


def main():
    adam: Adam = Adam('real')
    initial_info: AdamInfo = adam.connect('localhost', 1883, rate=10)

    left_configuration: Configuration = Configuration(1.5708, 0.58905, -0.098, -1.5708, 3.1415, 0.0)
    # left_configuration: Configuration = Configuration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    right_configuration: Configuration = Configuration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for _ in range(10):
        left_configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.0, 0.1)
        right_configuration -= Configuration(0.0, 0.0, 0.0, 0.0, 0.0, 0.1)

        adam.right_manipulator.set_to(right_configuration)
        adam.left_manipulator.set_to(left_configuration)

        info: AdamInfo = adam.step()

        Logger.debug(info.left_manipulator.configuration)

    adam.close()


if __name__ == '__main__':
    main()
