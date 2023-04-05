import numpy as np

from adam_sim import Adam, DataManager
from adam_sim.entities import AdamInfo, Point

from vclog import Logger


def main():
    adam: Adam = Adam()
    initial_info: AdamInfo = adam.load()

    end_effector_positions_right: list[Point] = np.linspace(initial_info.left_manipulator.systems[-1].position, Point(1.0, -1.0, 1.0), 100).tolist()
    end_effector_positions_left: list[Point] = np.linspace(initial_info.left_manipulator.systems[-1].position, Point(1.0, 1.0, 1.0), 100).tolist()

    for point_right, point_left in zip(end_effector_positions_right, end_effector_positions_left):

        adam.right_manipulator.move_to(Point(*point_right))
        adam.left_manipulator.move_to(Point(*point_left))

        info: AdamInfo = adam.step()

        adam.render()

    adam.close()


if __name__ == '__main__':
    main()
