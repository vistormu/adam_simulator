import numpy as np

from typing import NamedTuple


class Configuration(NamedTuple):
    q1: float
    q2: float
    q3: float
    q4: float
    q5: float
    q6: float

    def __repr__(self) -> str:
        return f'({self.q1:.4f}, {self.q2:.4f}, {self.q3:.4f}, {self.q4:.4f}, {self.q5:.4f}, {self.q6:.4f})'

    def __add__(self, other):
        return Configuration(self.q1+other.q1,
                             self.q2+other.q2,
                             self.q3+other.q3,
                             self.q4+other.q4,
                             self.q5+other.q5,
                             self.q6+other.q6)

    def __sub__(self, other):
        return Configuration(self.q1-other.q1,
                             self.q2-other.q2,
                             self.q3-other.q3,
                             self.q4-other.q4,
                             self.q5-other.q5,
                             self.q6-other.q6)

    def __mul__(self, scalar: float):
        return Configuration(self.q1*scalar,
                             self.q2*scalar,
                             self.q3*scalar,
                             self.q4*scalar,
                             self.q5*scalar,
                             self.q6*scalar)

    def __rmul__(self, scalar: float):
        return Configuration(self.q1*scalar,
                             self.q2*scalar,
                             self.q3*scalar,
                             self.q4*scalar,
                             self.q5*scalar,
                             self.q6*scalar)

    def __truediv__(self, scalar: float):
        return Configuration(self.q1/scalar,
                             self.q2/scalar,
                             self.q3/scalar,
                             self.q4/scalar,
                             self.q5/scalar,
                             self.q6/scalar)

    def to_degrees(self):
        return self*180/np.pi

    def to_radians(self):
        return self*np.pi/180

    def to_numpy(self) -> np.ndarray:
        return np.array(self)
