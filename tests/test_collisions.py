import numpy as np

from vclog import Logger
from adam_sim import Adam
from adam_sim.entities import AdamInfo, Configuration


def main():
    adam: Adam = Adam()
    initial_info: AdamInfo = adam.load('tests/adam_scene/scene.xml')

    configuration: Configuration = initial_info.left_manipulator.configuration
    configuration_list: list[Configuration] = [configuration]

    self_collision_counter: int = 0
    env_collision_counter: int = 0

    for _ in range(1000):
        adam.render()

        adam.left_manipulator.set_to(configuration)
        adam.right_manipulator.set_to(initial_info.right_manipulator.configuration)

        info: AdamInfo = adam.step()

        if info.left_manipulator.collision.self_collision:
            self_collision_counter += 1
            Logger.warning(f'self collision: {self_collision_counter}', flush=True)

        if info.left_manipulator.collision.env_collision:
            env_collision_counter += 1
            Logger.error(f'env collision: {env_collision_counter}', flush=True)

        configuration += Configuration(0.0, 0.005, 0.0, 0.0, 0.0, 0.0)
        configuration_list.append(configuration)

    adam.close()

    # Standalone function
    env_collisions, self_collisions = adam.check_collisions(configuration_list, [initial_info.right_manipulator.configuration]*len(configuration_list))

    Logger.debug(np.sum(self_collisions) - self_collision_counter)
    Logger.debug(np.sum(env_collisions) - env_collision_counter)

    adam.close()


if __name__ == '__main__':
    main()
