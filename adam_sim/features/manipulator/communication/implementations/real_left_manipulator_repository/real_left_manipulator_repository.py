from .use_cases import MQTTClient
from ....repository import ManipulatorRepository
from ......entities import Configuration, System, Velocity, Acceleration, Point, Vector
from ....entities import Collision, ManipulatorInfo


class RealLeftManipulatorRepository(ManipulatorRepository):
    def init(self, host: str, port: int) -> None:
        self.client = MQTTClient(host, port)

    def set_configuration(self, configuration: Configuration) -> None:
        self.client.publish_configuration(configuration)

    def get_configuration(self) -> Configuration:
        return self.client.get_configuration()

    def set_velocity(self, velocity: Velocity) -> None:
        return super().set_velocity(velocity)

    def get_velocity(self) -> Velocity:
        return Velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def set_acceleration(self, acceleration: Acceleration) -> None:
        return super().set_acceleration(acceleration)

    def get_acceleration(self) -> Acceleration:
        return Acceleration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def get_collisions(self) -> Collision:
        return Collision.empty()

    def get_systems(self) -> list[System]:
        return []

    def get_info(self) -> ManipulatorInfo:
        systems: list[System] = self.get_systems()
        end_effector: System = System(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0))
        configuration: Configuration = self.get_configuration()
        velocity: Velocity = self.get_velocity()
        acceleration: Acceleration = self.get_acceleration()
        collision: Collision = self.get_collisions()

        return ManipulatorInfo(systems=systems,
                               end_effector=end_effector,
                               configuration=configuration,
                               velocity=velocity,
                               acceleration=acceleration,
                               collision=collision)

    def close(self) -> None:
        self.client.close()
