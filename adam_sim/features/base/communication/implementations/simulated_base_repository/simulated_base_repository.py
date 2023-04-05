from ....repository import BaseRepository
from ......entities import Point, Vector
from ....entities import BaseInfo


class SimulatedBaseRepository(BaseRepository):
    def init(self, data):
        self.data = data

    def get_position(self) -> Point:
        return super().get_position()

    def get_info(self) -> BaseInfo:
        return BaseInfo(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0))
