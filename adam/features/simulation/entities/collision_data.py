from dataclasses import dataclass

from .collision import Collision


@dataclass
class CollisionData:
    '''
    the class CollisitonData contains the information of all collisions produced

    Attributes
    ----------
    left_manipulator : ~.entities.Collision
        the collision data of the left manipulator

    right_manipulator : ~.entities.Collision
        the collision data of the right manipulator
    '''
    left_manipulator: Collision
    right_manipulator: Collision
