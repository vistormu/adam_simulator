from ...entities import Point, Vector
from .communication import get_base_repository
from .repository import BaseRepository


class Base:
    '''
    TMP
    '''

    def __init__(self, id: str) -> None:
        self._repository: BaseRepository = get_base_repository(id)

    def move_to(self, position: Point, orientation: Vector) -> None:
        pass
