from dataclasses import dataclass

from ...manipulator.entities import ManipulatorInfo
from ...base.entities import BaseInfo


@dataclass
class AdamInfo:
    '''
    the ADAM Info class is a dataclass that contains all the information of the simulation

    Attributes
    ----------
    left_manipulator : ~.entities.ManipulatorInfo
        the data referring to the left manipulator of the robot

    right_manipulator : ~.entities.ManipulatorInfo
        the data referring to the right manipulator of the robot

    base : ~.entities.BaseInfo
        the data referring to the base of the robot
    '''
    left_manipulator: ManipulatorInfo
    right_manipulator: ManipulatorInfo
    base: BaseInfo
