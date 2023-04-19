from ...entities import Point, Configuration, System, Velocity
from .repository import ManipulatorRepository
from .communication import get_manipulator_repository
from .use_cases import Farm, ControlVisualizer


class Manipulator:
    '''
    the `Manipulator` class contains all the methods to control and render the manipulator either in simulation or in real life.

    Methods
    -------
    move_to:
        moves the manipulator to the specified position

    set_to:
        sets the manipulator to the specified configuration

    set_velocity_to:
        sets the manipulator velocity to the specified vector

    control:
        control the manipulator via an interface
    ''' 
    def _init_repository(self, id: str, *args, **kwargs) -> None:
        self._repository: ManipulatorRepository = get_manipulator_repository(id)
        self._control_visualizer: ControlVisualizer = ControlVisualizer(id)
        
        self._repository.init(*args, **kwargs)

    def move_to(self, point: Point) -> None:
        '''
        moves the manipulator to the specified position

        Parameters
        ----------
        point : ~.entities.Point
            the position to move the manipulator's end effector to
        '''
        systems: list[System] = self._repository.get_systems()
        target: System = System(point, systems[-1].x_axis, systems[-1].y_axis, systems[-1].z_axis)
        configuration: Configuration | None = Farm.iterate(systems, target)

        if configuration is None:
            return

        self._repository.set_configuration(configuration)

    def set_to(self, configuration: Configuration) -> None:
        '''
        sets the manipulator to the specified configuration

        Parameters
        ----------
        configuration : ~.entities.Configuration
            the configuration of the manipulator
        '''
        self._repository.set_configuration(configuration)

    def set_velocity_to(self, velocity: Velocity) -> None:
        '''
        sets the manipulator velocity to the specified vector
        
        Parameters
        ----------
        velocity : ~.entities.Velocity
            the velocity of the manipulator
        '''
        self._repository.set_velocity(velocity)

    def control(self) -> None:
        '''
        control the manipulator via an interface
        '''
        self._control_visualizer.init(self._repository.get_configuration())
        
        self._repository.set_configuration(self._control_visualizer.get())
        
    