from dataclasses import dataclass

from .collision import Collision


@dataclass
class CollisionData:
    left_manipulator: Collision
    right_manipulator: Collision
