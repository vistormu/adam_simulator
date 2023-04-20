from abc import ABC, abstractmethod


class AdamRepository(ABC):
    def __init__(self) -> None:
        self.model = None
        self.data = None

    @abstractmethod
    def init(self, *args, **kwargs) -> None:
        pass

    @abstractmethod
    def step(self) -> None:
        pass

    @abstractmethod
    def close(self) -> None:
        pass
