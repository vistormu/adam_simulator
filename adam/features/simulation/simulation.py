import pkg_resources
import numpy as np

from distutils.dir_util import copy_tree

from .entities import Configuration, AdamInfo
from .use_cases import DataManager, ControlVisualizer, Viewer, Controller


class Simulation:
    '''
    the Simulation class contains all the methods to control and render the ADAM robot.

    Methods
    -------
    load_scene:
        loads a given MuJoCo scene. This method is mandatory to call

    step:
        step on the simulation's dynamics

    control:
        control the manipulators via sliders

    render:
        render the scene

    close:
        close all the renderers

    extend_collisions:
        extends the collision dictionary

    check_collisions:
        checks all the collisions in a given configuration path

    export_scene:
        exports the default ADAM scene to a given directory
    '''

    def __init__(self) -> None:
        self.data_manager: DataManager = DataManager()
        self.left_control_visualizer: ControlVisualizer = ControlVisualizer('Left Manipulator')
        self.right_control_visualizer: ControlVisualizer = ControlVisualizer('Right Manipulator')
        self.controller: Controller = Controller()
        self.viewer: Viewer = Viewer()

        self.left_control_mode: bool = False
        self.right_control_mode: bool = False

    def load_scene(self, filename: str | None = None) -> AdamInfo:
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
        if filename is None:
            filename = pkg_resources.resource_filename('adam', 'assets/scene.xml')

        self.controller.init(filename)

        return self.data_manager.get(self.controller.data)

    def extend_collisions(self, collision_dict: dict[int, str]) -> None:
        '''
        it extends the collisions dictionary to incorporate new geometries

        Parameters
        ----------
        collision_dict : dict[int, str]
            the collision dictionary to incorporate

        Notes
        -----
        in order to get the corresponding ids, please refer to the geometry_list.md on the GitHub page
        '''
        self.data_manager.collision_detector.extend_collisions(collision_dict)

    def check_collisions(self, left_configuration_list: list[Configuration], right_configuration_list: list[Configuration]) -> tuple[np.ndarray, np.ndarray]:
        '''
        a method to directly calculate the collisions on a given set of configurations

        Parameters
        ----------
        left_configuration_list : list[~.entities.Configuration]
            the list of configurations for the left manipulator

        right_configuration_list : list[~.entities.Configuration]
            the list of configurations for the right manipulator

        Returns
        -------
        out : tuple[np.ndarray, np.ndarray]
            a tuple containing the self collisions and the environment collisions respectively. A given value of the array is 1 if a collision has been detected
        '''
        self_collision_array: np.ndarray = np.zeros(len(left_configuration_list))
        env_collision_array: np.ndarray = np.zeros(len(left_configuration_list))
        for i, (left_configuration, right_configuration) in enumerate(zip(left_configuration_list, right_configuration_list)):
            info: AdamInfo = self.step(left_configuration, right_configuration)
            if info.left_manipulator.collision.self_collision or info.right_manipulator.collision.self_collision:
                self_collision_array[i] = 1
            if info.left_manipulator.collision.env_collision or info.right_manipulator.collision.env_collision:
                env_collision_array[i] = 1

        return self_collision_array, env_collision_array

    def step(self, left_configuration: Configuration, right_configuration: Configuration) -> AdamInfo:
        '''
        a step forward on the dynamics of the simulation

        Parameters
        ----------
        left_configuration : ~.entities.Configuration
            the configuration to send to the left manipulator

        right_configuration : ~.entities.Configuration
            the configuration to send to the right manipulator

        Returns
        -------
        out : ~.entities.AdamInfo
            the simulation data
        '''
        self.controller.set_configuration(left_configuration, right_configuration)
        self.controller.step()

        return self.data_manager.get(self.controller.data)

    def control(self, mode: str = 'both') -> AdamInfo:
        '''
        a method to control directly the manipulators via sliders

        Parameters
        ----------
        mode : str, optional
            indicates the manipulator to control. Possible values: 'left', 'right', 'both'. 'both' by default

        Returns
        -------
        out : ~.entities.AdamInfo
            the simulation data
        '''
        if mode == 'left':
            self.left_control_mode = True
        elif mode == 'right':
            self.right_control_mode = True
        elif mode == 'both':
            self.left_control_mode = True
            self.right_control_mode = True

        left_configuration: Configuration = self.controller.get_left_configuration()
        right_configuration: Configuration = self.controller.get_right_configuration()

        if self.left_control_visualizer.is_active:
            left_configuration = Configuration(*self.left_control_visualizer.get())

        if self.right_control_visualizer.is_active:
            right_configuration = Configuration(*self.right_control_visualizer.get())

        self.controller.set_configuration(left_configuration, right_configuration)
        self.controller.step()

        return self.data_manager.get(self.controller.data)

    def render(self, fps: int | None = None, hide_menu: bool = False) -> None:
        '''
        a method to render the simulation and the configuration sliders

        Parameters
        ----------
        fps : int, optional
            the fps of the simulation. None by default
        '''
        if not self.viewer.is_active:
            self.viewer.init(self.controller.model, self.controller.data)

        if self.left_control_mode and not self.left_control_visualizer.is_active:
            self.left_control_visualizer.init(self.controller.get_left_configuration().to_numpy())

        if self.right_control_mode and not self.right_control_visualizer.is_active:
            self.right_control_visualizer.init(self.controller.get_right_configuration().to_numpy())

        self.left_control_visualizer.render(60)
        self.right_control_visualizer.render(60)
        self.viewer.render(fps, hide_menu)

    def close(self) -> None:
        '''
        a method to close the rendering if any exists
        '''
        if self.viewer.is_active:
            self.viewer.close()

        if self.left_control_visualizer.is_active:
            self.left_control_visualizer.close()

        if self.right_control_visualizer.is_active:
            self.right_control_visualizer.close()

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
        filename = pkg_resources.resource_filename('adam', 'assets/')
        copy_tree(filename, destination)
