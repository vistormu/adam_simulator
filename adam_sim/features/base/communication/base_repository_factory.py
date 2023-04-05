from .implementations import RealBaseRepository, SimulatedBaseRepository
from ..repository import BaseRepository


def get_base_repository(id: str) -> BaseRepository:
    if id == 'real':
        return RealBaseRepository()
    elif id == 'simulated':
        return SimulatedBaseRepository()

    raise ValueError(f'Unknown base repository id: {id}')
