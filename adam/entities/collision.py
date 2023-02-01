from typing import NamedTuple

class Collision(NamedTuple):
    shoulder: bool
    upper_arm: bool
    forearm: bool
    wrist_1: bool
    wrist_2: bool
    wrist_3: bool
    