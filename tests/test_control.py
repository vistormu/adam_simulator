from adam_sim import Adam
from adam_sim.entities import AdamInfo


def main():
    adam: Adam = Adam()

    initial_info: AdamInfo = adam.load()

    while True:
        adam.render()

        adam.left_manipulator.control()
        adam.right_manipulator.control()

        info: AdamInfo = adam.step()


if __name__ == '__main__':
    main()
