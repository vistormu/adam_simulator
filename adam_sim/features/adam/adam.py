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
    connect:
        connects to a given host and port

    load:
        loads a given MuJoCo scene. This method is mandatory to call

    step:
        advance one step in the simulation

    render:
        renders the scene

    close:
        closes all the visualizers

    check_collisions:
        checks if there are any collisions in the scene

    export_scene:
        exports the scene to the specified directory
    '''

    def __init__(self) -> None:
        # Features
        self.base: Base = Base()
        self.left_manipulator: Manipulator = Manipulator()
        self.right_manipulator: Manipulator = Manipulator()

        # Use cases
        self._viewer: Viewer = Viewer()

        # Repository
        self._repository: AdamRepository = None  # type: ignore

        # Variables
        self._mode: str = ''

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
        self._mode = 'real'

        # Initialize repositories
        self._repository = get_adam_repository('real')
        self._repository.init(host, port, rate)

        self.left_manipulator._init_repository('left_real', host, port)
        self.right_manipulator._init_repository('right_real', host, port)
        self.base._init_repository('real', host, port)

        # Get initial info
        self._repository.step()

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
        self._mode = 'sim'

        if filename is None:
            filename = pkg_resources.resource_filename('adam_sim', RESOURCE_FILE + 'scene.xml')

        # Initialize repositories
        self._repository = get_adam_repository('simulated')
        self._repository.init(filename)

        self.left_manipulator._init_repository('left_simulated', self._repository.data)
        self.right_manipulator._init_repository('right_simulated', self._repository.data)
        self.base._init_repository('simulated', self._repository.data)

        # Get initial info
        self._repository.step()

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
        if self._mode != 'sim':
            return

        self._viewer.init(self._repository.model, self._repository.data)

        self.left_manipulator._control_visualizer.render(60)
        self.right_manipulator._control_visualizer.render(60)
        self._viewer.render(hide_menu)

    def close(self) -> None:
        '''
        closes all the visualizers and connections
        '''
        self.left_manipulator._control_visualizer.close()
        self.right_manipulator._control_visualizer.close()
        self._viewer.close()

        self._repository.close()
        self.left_manipulator._repository.close()
        self.right_manipulator._repository.close()
        self.base._repository.close()

    def check_collisions(self, left_configurations: list[Configuration], right_configurations: list[Configuration]) -> tuple[np.ndarray, np.ndarray]:
        '''
        checks wether there are self collisions or environment collisions

        Parameters
        ----------
        left_configurations : list[~.entities.Configuration]
            the configurations of the left manipulator

        right_configurations : list[~.entities.Configuration]
            the configurations of the right manipulator

        Returns
        -------
        out : tuple[np.ndarray, np.ndarray]
            the first element is an array of the environment collisions and the second element is an array of the self collisions

        Raises
        ------
        Exception
            if the robot is not in simulation mode
        '''
        if self._mode != 'sim':
            raise Exception('The check_collisions method is only available in simulation mode')

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

        Parameters
        ----------
        destination : str
            the directory in which to export the scene

        Notes
        -----
        Remember that the destination must end with '/'
        '''
        filename = pkg_resources.resource_filename('adam_sim', RESOURCE_FILE)
        copy_tree(filename, destination + 'adam_scene/')
