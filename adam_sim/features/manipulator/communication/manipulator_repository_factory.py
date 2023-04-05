from ..repository import ManipulatorRepository
from .implementations import RealLeftManipulatorRepository, RealRightManipulatorRepository, SimulatedLeftManipulatorRepository, SimulatedRightManipulatorRepository


def get_manipulator_repository(id: str) -> ManipulatorRepository:
    if id == 'left_simulated':
        return SimulatedLeftManipulatorRepository()
    elif id == 'right_simulated':
        return SimulatedRightManipulatorRepository()
    elif id == 'left_real':
        return RealLeftManipulatorRepository()
    elif id == 'right_real':
        return RealRightManipulatorRepository()

    raise ValueError(f'Unknown manipulator repository id: {id}')
