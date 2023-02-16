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

## Minimal Working Examples

In this example, ADAM moves the left and right wrist_2 continuously.

```python
from adam import Simulation
from adam.entities import Configuration, Data


def main():
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene()

    left_configuration: Configuration = initial_data.configuration.left_manipulator
    right_configuration: Configuration = initial_data.configuration.right_manipulator

    while sim.is_alive:
        left_configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)
        right_configuration -= Configuration(0.0, 0.0, 0.0, 0.0, 0.1, 0.0)

        sim.render()
        data: Data = sim.step(left_configuration, right_configuration)

    sim.close()


if __name__ == '__main__':
    main()
```

In this example, a list of configurations are loaded and tested on the left manipulator.

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

## API Reference
The developed API aims to wrap the MuJoCo API in order to simply move the manipulators of ADAM. The API is object-oriented and gives type annotations.

**NOTE:** the documentation is still in progress.

### Data class
The ```Data``` class contains all the information related to the state of the manipulators. Currently the information offered is of the current configuration and the collision info.

```python
class Data:
    configuration: ConfigurationData
    collision: CollisionData
```

### Configuration Data class

The ```ConfigurationData``` class contains the configuration data of the left are right manipulator separately.

```python
class ConfigurationData:
    left_manipulator: Configuration
    right_manipulator: Configuration
```

### Configuration class
The ```Configuration``` class contains the information of a configuration of the manipulator.

```python
class Configuration(NamedTuple):
    q1: float
    q2: float
    q3: float
    q4: float
    q5: float
    q6: float

    def to_degrees(self) -> Configuration
    def to_radians(self) -> Configuration
    def to_numpy(self) -> np.ndarray
```

### Collision Data class
The ```CollisionData``` class contains the collision data of the left and right manipulator separately.

```python
class CollisionData:
    left_manipulator: Collision
    right_manipulator: Collision
```

### Collision class
The ```Collision``` class contains all the information of the collision of a manipulator.

```python
class Collision(NamedTuple):
    collided: bool
    vector: list[bool]
    shoulder: list[str]
    upper_arm: list[str]
    forearm: list[str]
    wrist_1: list[str]
    wrist_2: list[str]
    wrist_3: list[str]
```

### Simulation class
The ```Simulation``` class offers all the methods related to the simulation.

```python
class Simulation:
    is_alive: bool

    def load_scene(self, scene: str) -> Data
    def set_view(self, center: tuple[float, float, float], azimuth: float, elevation: float, distance: float) -> None
    def extend_collisions(collision_dict: dict[int, str]) -> None:
    def step(self, left_configuration: Configuration, right_configuration: Configuration) -> Data
    def render(self, *, fps: int = 15) -> None
    def close(self) -> None
```

### Configurations Manager class
The ```ConfigurationsManager``` class loads and saves a list of ```Configuration``` from a ```.csv``` file from a given directory.

```python
class ConfigurationsManager:
    @staticmethod
    def load(filename: str) -> list[Configuration]

    @staticmethod
    def save(filename: str, configuration_list: list[Configuration]) -> None
```