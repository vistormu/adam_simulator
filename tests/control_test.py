from adam_sim import Adam
from adam_sim.entities import AdamInfo
from vclog import Logger


def main():
    adam: Adam = Adam()
    initial_info: AdamInfo = adam.load()

    while True:
        adam.render()
        adam.left_manipulator.control()
        adam.right_manipulator.control()
        info: AdamInfo = adam.step()

        Logger.warning(info.left_manipulator.collision.vector, ' ', info.left_manipulator.collision.collided, flush=True)


if __name__ == '__main__':
    main()
