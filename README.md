# ADAM simulator

ADAM (Autonomous Domestic Ambidextrous Manipulator) is a mobile robot manipulator consisting of a base with two Degrees of Freedom (DoF) and two Universal Robots UR3 of 6 DoF each.

The simulation was built using [MuJoCo](https://mujoco.org/), a free and open source physics engine designed from the ground up for the purpose of model-based optimization, and in particular optimization through contacts.

## Installation

Follow the next steps for installing the simulation on your device.

**Requierements:**
- Ubuntu
- Python 3.10.0 or higher

### Install miniconda (highly-recommended)
It is highly recommended to install all the dependencies on a new virtual environment. For more information check the conda documentation for [installation](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) and [environment management](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html). For creating the environment use the following commands on the terminal.

```bash
conda create -n adam python==3.10.8
conda activate adam
```
### Install from pip
The ADAM simulator is available as a pip package. For installing it just use:
```
pip install adam-sim
```

### Install from source
Firstly, clone the repository in your system.
```bash
git clone https://github.com/vistormu/adam_simulator.git
```

Then, enter the directory and install the required dependencies
```bash
cd adam_simulator
pip install -r requirements.txt
```

## API Reference
Visit TMP

## Minimal Working Examples

### Basic Usage

In this example, ADAM moves the left and right wrist_2 continuously.

```python
from adam import Simulation
from adam.entities import Configuration, Data


def main():
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene()

    left_configuration: Configuration = initial_data.configuration.left_manipulator
    right_configuration: Configuration = initial_data.configuration.right_manipulator

    for _ in range(100):
        left_configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)
        right_configuration -= Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)

        sim.render()
        data: Data = sim.step(left_configuration, right_configuration)

    sim.close()


if __name__ == '__main__':
    main()
```

### ConfigurationsManager example

In this example, the test configurations are loaded to the left manipulator.

```python
from adam import Simulation, ConfigurationsManager
from adam.entities import Configuration, Data


def main():
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene()

    configuration_list: list[Configuration] = ConfigurationsManager.load('test')

    for configuration in configuration_list:
        sim.render()

        data: Data = sim.step(configuration, initial_data.configuration.right_manipulator)

    sim.close()


if __name__ == '__main__':
    main()
```

### MapMaker example

In this example, various bodies are created and then added to the scene

```python
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
    map_maker.make()

    Simulation.export_scene(directory_path)
    map_maker.add_to(directory_path + 'scene.xml')

    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene('tests/assets/scene.xml')

    sim.extend_collisions({77: 'table'})

    for _ in range(1000):
        sim.step(initial_data.configuration.left_manipulator, initial_data.configuration.right_manipulator)
        sim.render()


if __name__ == '__main__':
    main()
```