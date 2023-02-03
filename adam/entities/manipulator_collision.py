from typing import NamedTuple


class Collision(NamedTuple):
    shoulder: list[str]
    upper_arm: list[str]
    forearm: list[str]
    wrist_1: list[str]
    wrist_2: list[str]
    wrist_3: list[str]
