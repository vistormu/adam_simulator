from abc import ABC, abstractmethod

from ..entities import AdamInfo


class AdamRepository(ABC):
    def __init__(self) -> None:
        self.model = None
        self.data = None

    @abstractmethod
    def init(self, filename: str) -> None:
        pass

    @abstractmethod
    def step(self) -> None:
        pass
