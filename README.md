# ADAM Simulator

<p align="center">
    <a href="https://github.com/vistormu/adam_simulator">
        <img src="https://raw.githubusercontent.com/vistormu/adam_simulator/master/docs/assets/adam.png">
    </a>
</p>

[![pypi version](https://img.shields.io/pypi/v/adam-sim?logo=pypi)](https://pypi.org/project/adam-sim/)
[![MIT License](https://img.shields.io/badge/license-MIT-blue.svg?style=flat)](http://choosealicense.com/licenses/mit/)
[![docs](https://badgen.net/badge/readthedocs/documentation/blue)](https://adam-simulator.readthedocs.io/en/latest/)

ADAM (Autonomous Domestic Ambidextrous Manipulator) is a mobile robot manipulator consisting of a base with two Degrees of Freedom (DoF) and two Universal Robots UR3 of 6 DoF each.

The simulation was built using [MuJoCo](https://mujoco.org/), a free and open source physics engine designed from the ground up for the purpose of model-based optimization, and in particular optimization through contacts.

## Installation

Follow the next steps for installing the simulation on your device.

**Requierements:**
- Python 3.10.0 or higher

> **Note**: The Adam Simulator works on Linux, Windows and Mac.

### Install miniconda (highly-recommended)
It is highly recommended to install all the dependencies on a new virtual environment. For more information check the conda documentation for [installation](https://conda.io/projects/conda/en/latest/user-guide/install/index.html) and [environment management](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html). For creating the environment use the following commands on the terminal.

```bash
conda create -n adam python==3.10.9
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


### Installation for the communication
The communication uses mosquitto as a broker. For installing it on your system, follow the instructions on the [mosquitto website](https://mosquitto.org/download/).


It is also necessary to install docker. For more information check the [docker documentation](https://docs.docker.com/get-docker/).

## Documentation
The official documentation of the package is available on [Read the Docs](https://adam-simulator.readthedocs.io/en/latest/). Here you will find the [installation instructions](https://adam-simulator.readthedocs.io/en/latest/src/installation.html), the [API reference](https://adam-simulator.readthedocs.io/en/latest/src/api_reference.html) and some [minimal working examples](https://adam-simulator.readthedocs.io/en/latest/src/examples.html).
