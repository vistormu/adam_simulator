import pkg_resources
import numpy as np

from distutils.dir_util import copy_tree

from ..base import Base
from ..manipulator import Manipulator
from ...entities import Configuration
from .entities import AdamInfo
from .use_cases import Viewer
from .communication import get_adam_repository
from .repository import AdamRepository

RESOURCE_FILE: str = 'features/adam/communication/implementations/simulated_adam_repository/assets/'


class Adam:
    '''
    The `Adam` class contains all the methods to control and render the ADAM robot either in simulation or in real life.

    Attributes
    ----------
    base : ~.entities.Base
        the base of the robot

    left_manipulator : ~.entities.Manipulator
        the left manipulator of the robot

    right_manipulator : ~.entities.Manipulator
        the right manipulator of the robot

    Methods
    -------
    load:
        loads a given MuJoCo scene. This method is mandatory to call

    step:
        advance one step in the simulation

    render:
        renders the scene

    close:
        closes all the visualizers

    export_scene:
        exports the scene to the specified directory
    '''

    def __init__(self, mode: str = 'simulated') -> None:
        '''
        Parameters
        ----------
        mode : str, optional
            the mode in which to run the simulation. It can be either 'simulated' or 'real'. By default it is 'simulated'
        '''
        # Features
        self.base: Base = Base(mode)
        self.left_manipulator: Manipulator = Manipulator('left_' + mode)
        self.right_manipulator: Manipulator = Manipulator('right_' + mode)

        # Use cases
        self._viewer: Viewer = Viewer()

        # Repository
        self._repository: AdamRepository = get_adam_repository(mode)

        # Variables
        self._mode: str = mode

    def connect(self, host: str, port: int, rate: int = 30) -> AdamInfo:
        '''
        connects to a given host and port

        Parameters
        ----------
        host : str
            the host to connect to

        port : int
            the port to connect to
        '''
        if self._mode != 'real':
            raise Exception('The connect method is only available in real mode')

        self._repository.init(host, port, rate)

        self.left_manipulator._repository.init(host, port)
        self.right_manipulator._repository.init(host, port)
        self.base._repository.init(host, port)

        self.step()

        left_manipulator_info = self.left_manipulator._repository.get_info()
        right_manipulator_info = self.right_manipulator._repository.get_info()
        base_info = self.base._repository.get_info()

        return AdamInfo(left_manipulator_info, right_manipulator_info, base_info)

    def load(self, filename: str | None = None) -> AdamInfo:
        '''
        loads a given MuJoCo scene.xml

        Parameters
        ----------
        filename : str, optional
            the path to the scene.xml. By default it loads the integrated ADAM scene

        Returns
        -------
        out : ~.entities.AdamInfo
            the initial info of the robot
        '''
        if self._mode != 'simulated':
            raise Exception('The load method is only available in simulation mode')

        if filename is None:
            filename = pkg_resources.resource_filename('adam_sim', RESOURCE_FILE + 'scene.xml')

        self._repository.init(filename)
        self.left_manipulator._repository.init(self._repository.data)
        self.right_manipulator._repository.init(self._repository.data)
        self.base._repository.init(self._repository.data)

        self.step()

        left_manipulator_info = self.left_manipulator._repository.get_info()
        right_manipulator_info = self.right_manipulator._repository.get_info()
        base_info = self.base._repository.get_info()

        return AdamInfo(left_manipulator_info, right_manipulator_info, base_info)

    def step(self) -> AdamInfo:
        '''
        advance one step in the simulation

        Returns
        -------
        out : ~.entities.AdamInfo
            the info of the robot at the current step
        '''
        self._repository.step()

        left_manipulator_info = self.left_manipulator._repository.get_info()
        right_manipulator_info = self.right_manipulator._repository.get_info()
        base_info = self.base._repository.get_info()

        return AdamInfo(left_manipulator_info, right_manipulator_info, base_info)

    def render(self, hide_menu: bool = True) -> None:
        '''
        renders the scene

        Parameters
        ----------
        hide_menu : bool, optional
            whether to hide the menu or not. By default it is True
        '''
        if self._mode != 'simulated':
            raise Exception('The render method is only available in simulation mode')

        self._viewer.init(self._repository.model, self._repository.data)

        self.left_manipulator._control_visualizer.render(60)
        self.right_manipulator._control_visualizer.render(60)
        self._viewer.render(hide_menu)

    def close(self) -> None:
        '''
        closes all the visualizers
        '''
        self.left_manipulator._control_visualizer.close()
        self.right_manipulator._control_visualizer.close()
        self._viewer.close()

    def check_collisions(self, left_configurations: list[Configuration], right_configurations: list[Configuration]) -> tuple[np.ndarray, np.ndarray]:
        '''
        checks wether there are self collisions or environment collisions

        Returns
        -------
        out : tuple[np.ndarray, np.ndarray]
            the first element is an array of the environment collisions and the second element is an array of the self collisions
        '''
        env_collisions: np.ndarray = np.zeros(len(left_configurations)).astype(bool)
        self_collisions: np.ndarray = np.zeros(len(left_configurations)).astype(bool)
        for i, (left_configuration, right_configuration) in enumerate(zip(left_configurations, right_configurations)):
            self.left_manipulator.set_to(left_configuration)
            self.right_manipulator.set_to(right_configuration)

            info: AdamInfo = self.step()

            if info.left_manipulator.collision.env_collision or info.right_manipulator.collision.env_collision:
                env_collisions[i] = True

            if info.left_manipulator.collision.self_collision or info.right_manipulator.collision.self_collision:
                self_collisions[i] = True

        return (env_collisions, self_collisions)

    @staticmethod
    def export_scene(destination: str) -> None:
        '''
        exports the default ADAM scene to a given directory

        destination : str
            the directory in which to export the scene

        Notes
        -----
        Remember that the destination must end with '/'
        '''
        filename = pkg_resources.resource_filename('adam_sim', RESOURCE_FILE)
        copy_tree(filename, destination + 'adam_scene/')
