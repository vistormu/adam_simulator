# ADAM simulator

ADAM (Autonomous Domestic Ambidextrous Manipulator) is a mobile robot manipulator consisting of a base with two Degrees of Freedom (DoF) and two Universal Robots UR3 of 6 DoF each.

MuJoCo

## Installation

### Install miniconda (highly-recommended)
It is highly recommended to install all the dependencies on a new virtual environment. For more information check the conda documentation for [installation](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) and [environment management](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html).

### Install from source
Firstly, clone the repository in your system.
```
git clone https://github.com/vistormu/adam_simulator.git
```

Then, enter the directory and install the required dependencies
```
cd adam_simulator
pip install -r requirements.txt
```

## Minimal Working Example
```
from adam import Simulation, Configuration, Data


def main():
    sim: Simulation = Simulation()
    initial_data: Data = sim.load_scene('scene')

    configuration: Configuration = initial_data.configuration

    while sim.is_alive:
        configuration += Configuration(0.0, 0.0, 0.0, 0.0, 0.001, 0.0)

        sim.render()
        data: Data = sim.step(configuration)

    sim.close()


if __name__ == '__main__':
    main()

```

## API
The API is object-oriented and consists of the following classes.

### Entities
The entities are the "brick" classes of the API.

#### Configuration
A tuple containing the information of the configuration of the robot.

#### Data
The data of the robot at each step of the simulation.

### Simulation
The Simulation class contains all the methods for the simulation.

#### load_scene
WIP

#### step
WIP

#### render
WIP

#### close
WIP