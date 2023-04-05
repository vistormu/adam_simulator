from adam_sim import Adam, DataManager
from adam_sim.entities import Configuration, AdamInfo, Point


def main():
    adam: Adam = Adam('simulated')
    initial_info: AdamInfo = adam.load()

    configuration_list: list[Configuration] = DataManager.load_configurations('tests/data/limits.csv')

    for configuration in configuration_list:
        adam.render()

        adam.left_manipulator.set_to(configuration)

        info: AdamInfo = adam.step()

    end_effector_positions: list[Point] = DataManager.load_end_effector_positions('tests/data/end_effector_positions_test.csv')

    for point in end_effector_positions:
        adam.render()

        adam.left_manipulator.move_to(point)

        info: AdamInfo = adam.step()

    adam.close()


if __name__ == '__main__':
    main()
