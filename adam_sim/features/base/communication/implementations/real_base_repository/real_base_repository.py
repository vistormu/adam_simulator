from ....repository import BaseRepository
from ......entities import Point
from ....entities import BaseInfo


class RealBaseRepository(BaseRepository):
    def init(self, *args, **kwargs):
        return super().init(*args, **kwargs)

    def get_position(self) -> Point:
        return super().get_position()

    def get_info(self) -> BaseInfo:
        return super().get_info()
