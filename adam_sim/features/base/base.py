from ...entities import Point, Vector
from .communication import get_base_repository
from .repository import BaseRepository


class Base:
    '''
    TMP
    '''

    def _init_repository(self, id: str, *args, **kwargs) -> None:
        self._repository: BaseRepository = get_base_repository(id)

        self._repository.init(*args, **kwargs)

    def move_to(self, position: Point, orientation: Vector) -> None:
        pass
