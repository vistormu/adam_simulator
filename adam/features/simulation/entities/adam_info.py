from dataclasses import dataclass

from .manipulator_data import ManipulatorData


@dataclass
class AdamInfo:
    '''
    the ADAM Info class is a dataclass that contains all the information of the simulation

    Attributes
    ----------
    left_manipulator : ~.entities.ConfigurationData
        the data referring to the left manipulator of the robot

    right_manipulator : ~.entities.CollisionData
        the data referring to the right manipulator of the robot
    '''
    left_manipulator: ManipulatorData
    right_manipulator: ManipulatorData
