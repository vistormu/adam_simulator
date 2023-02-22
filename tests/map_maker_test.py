from adam import Simulation, MapMaker
from adam.entities import Data, Cube, Box, Capsule, Cylinder, Sphere


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

    map_maker.export_scene(directory_path)
    map_maker.create_xml()
    map_maker.add_to_scene(directory_path + 'scene.xml')

    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene('tests/assets/scene.xml')

    for _ in range(1000):
        sim.step(initial_data.configuration.left_manipulator, initial_data.configuration.right_manipulator)
        sim.render()


if __name__ == '__main__':
    main()
