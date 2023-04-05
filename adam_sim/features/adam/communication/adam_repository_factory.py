from .implementations import RealAdamRepository, SimulatedAdamRepository
from ..repository import AdamRepository


def get_adam_repository(id: str) -> AdamRepository:
    if id == 'real':
        return RealAdamRepository()
    elif id == 'simulated':
        return SimulatedAdamRepository()

    raise ValueError(f'Unknown Adam repository id: {id}')
