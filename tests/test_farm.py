import numpy as np

from adam_sim import Adam
from adam_sim.entities import AdamInfo, Point


def main():
    adam: Adam = Adam()

    initial_info: AdamInfo = adam.load()

    x_path: np.ndarray = np.linspace(0.0, 1.0, 100)
    left_path: list[Point] = [initial_info.left_manipulator.end_effector.position + Point(x, 0.0, 0.0) for x in x_path]
    right_path: list[Point] = [initial_info.right_manipulator.end_effector.position + Point(x, 0.0, 0.0) for x in x_path]

    for left_point, right_point in zip(left_path, right_path):
        adam.render()

        adam.left_manipulator.move_to(left_point)
        adam.right_manipulator.move_to(right_point)

        info: AdamInfo = adam.step()

    adam.close()


if __name__ == '__main__':
    main()
