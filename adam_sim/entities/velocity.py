import numpy as np

from typing import NamedTuple


class Velocity(NamedTuple):
    '''
    the Velocity class is a NamedTuple that contains the values of the joint velocities. Unit: rad/s

    Attributes
    ----------
    w1 : float
        the value of the angular velocity of the first joint

    w2 : float
        the value of the angular velocity of the second joint

    w3 : float
        the value of the angular velocity of the third joint

    w4 : float
        the value of the angular velocity of the fourth joint

    w5 : float
        the value of the angular velocity of the fifth joint

    w6 : float
        the value of the angular velocity of the sixth joint
    '''
    w1: float
    w2: float
    w3: float
    w4: float
    w5: float
    w6: float

    def __repr__(self) -> str:
        return f'({self.w1:.4f}, {self.w2:.4f}, {self.w3:.4f}, {self.w4:.4f}, {self.w5:.4f}, {self.w6:.4f})'

    def __add__(self, other):
        return Velocity(self.w1+other.w1,
                        self.w2+other.w2,
                        self.w3+other.w3,
                        self.w4+other.w4,
                        self.w5+other.w5,
                        self.w6+other.w6)

    def __sub__(self, other):
        return Velocity(self.w1-other.w1,
                        self.w2-other.w2,
                        self.w3-other.w3,
                        self.w4-other.w4,
                        self.w5-other.w5,
                        self.w6-other.w6)

    def __mul__(self, other):
        return Velocity(self.w1*other,
                        self.w2*other,
                        self.w3*other,
                        self.w4*other,
                        self.w5*other,
                        self.w6*other)

    def __rmul__(self, other):
        return Velocity(self.w1*other,
                        self.w2*other,
                        self.w3*other,
                        self.w4*other,
                        self.w5*other,
                        self.w6*other)

    def __truediv__(self, other):
        return Velocity(self.w1/other,
                        self.w2/other,
                        self.w3/other,
                        self.w4/other,
                        self.w5/other,
                        self.w6/other)

    def __neg__(self):
        return Velocity(-self.w1,
                        -self.w2,
                        -self.w3,
                        -self.w4,
                        -self.w5,
                        -self.w6)

    def to_numpy(self) -> np.ndarray:
        '''
        converts a Velocity instance to a numpy array

        Returns
        -------
        np.ndarray
            a numpy array containing the values of the Velocity instance
        '''
        return np.array(self)
