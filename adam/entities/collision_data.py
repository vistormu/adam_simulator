from dataclasses import dataclass

from .collision import Collision


@dataclass
class CollisionData:
    left_manipulator_vector: list[bool]
    right_manipulator_vector: list[bool]
    left_manipulator_info: Collision
    right_manipulator_info: Collision
