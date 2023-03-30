import numpy as np

from vclog import Logger
from adam import Simulation
from adam.entities import AdamInfo, Configuration


def main():
    # Simulate
    sim: Simulation = Simulation()
    initial_info: AdamInfo = sim.load_scene('tests/assets/scene.xml')

    configuration: Configuration = initial_info.left_manipulator.configuration
    configuration_list: list[Configuration] = [configuration]

    self_collision_counter: int = 0
    env_collision_counter: int = 0
    for _ in range(500):
        sim.render(fps=120, hide_menu=True)
        info: AdamInfo = sim.step(configuration, initial_info.right_manipulator.configuration)

        if info.left_manipulator.collision.self_collision:
            self_collision_counter += 1
            Logger.warning(f'self collision: {self_collision_counter}', flush=True)

        if info.left_manipulator.collision.env_collision:
            env_collision_counter += 1
            Logger.error(f'env collision: {env_collision_counter}', flush=True)

        configuration += Configuration(0.0, 0.01, 0.0, 0.0, 0.0, 0.0)
        configuration_list.append(configuration)

    sim.close()

    # Standalone function
    sim: Simulation = Simulation()
    initial_info: AdamInfo = sim.load_scene('tests/assets/scene.xml')

    self_collisions, env_collisions = sim.check_collisions(configuration_list, [initial_info.right_manipulator.configuration]*len(configuration_list))

    Logger.debug(np.sum(self_collisions) - self_collision_counter)
    Logger.debug(np.sum(env_collisions) - env_collision_counter)

    sim.close()


if __name__ == '__main__':
    main()
