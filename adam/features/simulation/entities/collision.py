from typing import NamedTuple

class Collision(NamedTuple):
    collided: bool
    self_collision: bool
    env_collision: bool
    vector: list[bool]
    shoulder: list[str]
    upper_arm: list[str]
    forearm: list[str]
    wrist_1: list[str]
    wrist_2: list[str]
    wrist_3: list[str]
    
    @classmethod
    def empty(cls):
        return cls(False, False, False, [], [], [], [], [], [], [])
    