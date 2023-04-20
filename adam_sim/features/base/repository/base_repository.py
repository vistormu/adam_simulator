from abc import ABC, abstractmethod

from ....entities import Point
from ..entities import BaseInfo


class BaseRepository(ABC):
    @abstractmethod
    def init(self, *args, **kwargs):
        pass

    @abstractmethod
    def get_position(self) -> Point:
        pass

    @abstractmethod
    def get_info(self) -> BaseInfo:
        pass

    @abstractmethod
    def close(self) -> None:
        pass
