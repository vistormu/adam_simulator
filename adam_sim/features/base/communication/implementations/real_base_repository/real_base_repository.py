from ....repository import BaseRepository
from ......entities import Point, System, Vector
from ....entities import BaseInfo


class RealBaseRepository(BaseRepository):
    def init(self, *args, **kwargs):
        return super().init(*args, **kwargs)

    def get_position(self) -> Point:
        return super().get_position()

    def get_info(self) -> BaseInfo:
        return BaseInfo(position=Point(0.0, 0.0, 0.0),
                        system=System(Point(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0)),
                        velocity=Vector(0.0, 0.0, 0.0))

    def close(self) -> None:
        return super().close()
