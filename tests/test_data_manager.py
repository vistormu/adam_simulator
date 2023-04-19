from adam_sim import Adam, DataManager
from adam_sim.entities import Configuration, AdamInfo, Point


def main():
    adam: Adam = Adam()
    initial_info: AdamInfo = adam.load()

    configuration_list: list[Configuration] = DataManager.load_configurations('test')

    end_effector_positions: list[Point] = [initial_info.left_manipulator.systems[-1].position]

    for configuration in configuration_list:
        adam.render()

        adam.left_manipulator.set_to(configuration)
        info: AdamInfo = adam.step()

        end_effector_positions.append(info.left_manipulator.systems[-1].position)

    DataManager.save_points('tests/data/end_effector_positions_test.csv', end_effector_positions)

    end_effector_positions: list[Point] = DataManager.load_points('tests/data/end_effector_positions_test.csv')

    for point in end_effector_positions:
        adam.render()

        adam.left_manipulator.move_to(point)

        info: AdamInfo = adam.step()

    adam.close()


if __name__ == '__main__':
    main()
