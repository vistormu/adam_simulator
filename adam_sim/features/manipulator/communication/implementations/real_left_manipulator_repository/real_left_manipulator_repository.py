from ....repository import ManipulatorRepository
from ......entities import Configuration, System, Velocity, Acceleration, Point, Vector
from ....entities import Collision, ManipulatorInfo


class RealLeftManipulatorRepository(ManipulatorRepository):
    def init(self) -> None:
        return super().init()

    def set_configuration(self, configuration: Configuration) -> None:
        return super().set_configuration(configuration)

    def get_configuration(self) -> Configuration:
        return super().get_configuration()

    def set_velocity(self, velocity: Velocity) -> None:
        return super().set_velocity(velocity)

    def get_velocity(self) -> Velocity:
        return super().get_velocity()

    def set_acceleration(self, acceleration: Acceleration) -> None:
        return super().set_acceleration(acceleration)

    def get_acceleration(self) -> Acceleration:
        return super().get_acceleration()

    def get_collisions(self) -> Collision:
        return super().get_collisions()

    def get_systems(self) -> list[System]:
        return super().get_systems()

    def get_info(self) -> ManipulatorInfo:
        # systems: list[System] = self.get_systems()
        # end_effector: System = systems[-1]
        # configuration: Configuration = self.get_configuration()
        # velocity: Velocity = self.get_velocity()
        # acceleration: Acceleration = self.get_acceleration()
        # collision: Collision = self.get_collisions()

        systems: list[System] = []
        end_effector: System = System(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0))
        configuration: Configuration = Configuration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        velocity: Velocity = Velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        acceleration: Acceleration = Acceleration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        collision: Collision = Collision.empty()

        return ManipulatorInfo(systems=systems,
                               end_effector=end_effector,
                               configuration=configuration,
                               velocity=velocity,
                               acceleration=acceleration,
                               collision=collision)
