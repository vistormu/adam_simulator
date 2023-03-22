Examples
========

Basic Usage
-----------

In this example, ADAM moves the left and right wrist_2 continuously.

.. code-block:: Python

    from adam import Simulation
    from adam.entities import Configuration, AdamInfo


    def main():
        sim: Simulation = Simulation()
        initial_info: AdamInfo = sim.load_scene()

        left_configuration: Configuration = initial_info.left_manipulator.configuration
        right_configuration: Configuration = initial_info.right_manipulator.configuration

        for _ in range(100):
            left_configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)
            right_configuration -= Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)

            sim.render(hide_menu=True)
            info: AdamInfo = sim.step(left_configuration, right_configuration)

        sim.close()


    if __name__ == '__main__':
        main()


ConfigurationsManager Example
-----------------------------

In this example, the test configurations are loaded to the left manipulator.

.. code-block:: Python

    from adam import Simulation, ConfigurationsManager
    from adam.entities import Configuration, AdamInfo


    def main():
        sim: Simulation = Simulation()
        initial_info: AdamInfo = sim.load_scene()

        configuration_list: list[Configuration] = ConfigurationsManager.load('test')

        for configuration in configuration_list:
            sim.render()

            info: AdamInfo = sim.step(configuration, initial_info.right_manipulator.configuration)

        sim.close()


    if __name__ == '__main__':
        main()



MapMaker example
----------------

In this example, various bodies are created and then added to the scene

.. code-block:: Python
    
    from adam import Simulation, MapMaker
    from adam.entities import AdamInfo, Cube, Box, Capsule, Cylinder, Sphere


    def main():
        directory_path: str = 'tests/assets/'
        filename: str = 'obstacles.xml'

        map_maker: MapMaker = MapMaker(directory_path+filename)

        # Cube 1
        cube_1: Cube = Cube('cube_1')
        cube_1.set_geometry(size=0.5, position=(0.5, 0.0, 0.0))
        cube_1.set_dynamics(mass=1.0)
        cube_1.set_appearance('#2f2f2f')

        # Cube 2
        cube_2: Cube = Cube('cube_2')
        cube_2.set_geometry(size=0.5, position=(1.0, 0.0, 0.0))
        cube_2.set_dynamics(mass=1.0)
        cube_2.set_appearance(color='#b64545', alpha=0.5)

        # Cube 3
        cube_3: Cube = Cube('cube_3')
        cube_3.set_geometry(size=0.5, position=(1.0, 0.0, 0.5))
        cube_3.set_appearance('#c4c476')

        # Cube 4
        cube_4: Cube = Cube('cube_4')
        cube_4.set_geometry(size=0.5, position=(0.5, 0.0, 0.5))

        # Box
        box: Box = Box('box')
        box.set_geometry(size=(0.2, 0.2, 0.5), position=(1.0, 0.0, 1.0))
        box.set_appearance(color=(0.2, 0.2, 0.2), alpha=0.2)

        # Capsule
        capsule: Capsule = Capsule('capsule')
        capsule.set_geometry(size=(0.2, 0.5), position=(0.5, -0.5, 0.0))

        # Cylinder
        cylinder: Cylinder = Cylinder('cylinder')
        cylinder.set_geometry(size=(0.2, 0.5), position=(1.0, -0.5, 0.0))

        # Sphere
        sphere: Sphere = Sphere('sphere')
        sphere.set_geometry(size=0.25, position=(0.5, -1.0, 0.0))

        map_maker.add_bodies([cube_1, cube_2, cube_3, cube_4, box, capsule, cylinder, sphere])
        map_maker.make()

        Simulation.export_scene(directory_path)
        map_maker.add_to(directory_path + 'scene.xml')

        sim: Simulation = Simulation()
        initial_data: AdamInfo = sim.load_scene('tests/assets/scene.xml')

        sim.extend_collisions({77: 'table'})

        for _ in range(1000):
            sim.step(initial_data.left_manipulator.configuration, initial_data.right_manipulator.configuration)
            sim.render()


    if __name__ == '__main__':
        main()
