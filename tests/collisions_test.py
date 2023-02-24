import numpy as np

from adam import Simulation, Logger, MapMaker
from adam.entities import Data, Configuration, Box


def main():
    # Create map
    directory_path: str = 'tests/assets/'
    filename: str = 'obstacles.xml'

    map_maker: MapMaker = MapMaker(directory_path+filename)

    # Table
    table: Box = Box('table')
    table.set_geometry(size=(0.6, 1.5, 0.1), position=(0.0, 0.0, 1.5))
    table.set_appearance(color='#f2f2f2', alpha=0.3)

    # Table
    table_2: Box = Box('table_2')
    table_2.set_geometry(size=(0.6, 1.5, 0.1), position=(0.0, 0.0, 1.6))
    table_2.set_appearance(color='#b64545', alpha=0.3)

    map_maker.add_bodies([table, table_2])

    map_maker.export_scene(directory_path)
    map_maker.create_xml()
    map_maker.add_to_scene(directory_path + 'scene.xml')

    # Simulate
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene('tests/assets/scene.xml')

    configuration: Configuration = initial_data.configuration.left_manipulator
    configuration_list: list[Configuration] = [configuration]

    self_collision_counter: int = 0
    env_collision_counter: int = 0
    for _ in range(500):
        sim.render(fps=120)
        data: Data = sim.step(configuration, initial_data.configuration.right_manipulator)

        if data.collision.left_manipulator.self_collision:
            self_collision_counter += 1
            Logger.warning(f'self collision: {self_collision_counter}', flush=True)

        if data.collision.left_manipulator.env_collision:
            env_collision_counter += 1
            Logger.error(f'env collision: {env_collision_counter}', flush=True)

        configuration += Configuration(0.0, 0.01, 0.0, 0.0, 0.0, 0.0)
        configuration_list.append(configuration)

    sim.close()

    # Standalone function
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene('tests/assets/scene.xml')

    self_collisions, env_collisions = sim.check_collisions(configuration_list, [initial_data.configuration.right_manipulator]*len(configuration_list))

    Logger.debug(np.sum(self_collisions) - self_collision_counter)
    Logger.debug(np.sum(env_collisions) - env_collision_counter)

    sim.close()


if __name__ == '__main__':
    main()
