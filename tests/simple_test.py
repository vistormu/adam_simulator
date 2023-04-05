from adam_sim import Adam
from adam_sim.entities import Configuration, AdamInfo


def main():
    adam: Adam = Adam()

    initial_info: AdamInfo = adam.load()

    left_configuration: Configuration = initial_info.left_manipulator.configuration
    right_configuration: Configuration = initial_info.right_manipulator.configuration

    for _ in range(1000):
        adam.render()

        left_configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)
        right_configuration -= Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)

        adam.right_manipulator.set_to(right_configuration)
        adam.left_manipulator.set_to(left_configuration)

        info: AdamInfo = adam.step()

    adam.close()


if __name__ == '__main__':
    main()
