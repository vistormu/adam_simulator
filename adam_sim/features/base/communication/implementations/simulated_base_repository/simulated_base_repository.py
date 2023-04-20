from ....repository import BaseRepository
from ......entities import Point, Vector, System
from ....entities import BaseInfo


class SimulatedBaseRepository(BaseRepository):
    def init(self, data):
        self.data = data

    def get_position(self) -> Point:
        return super().get_position()

    def get_info(self) -> BaseInfo:
        return BaseInfo(position=Point(0.0, 0.0, 0.0),
                        system=System(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0)),
                        velocity=Vector(0.0, 0.0, 0.0))

    def close(self) -> None:
        return super().close()
