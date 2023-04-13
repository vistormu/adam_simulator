import numpy as np
from typing import NamedTuple


class Acceleration(NamedTuple):
    '''
    the Acceleration class is a NamedTuple that contains the values of the joint accelerations. Unit: rad/s^2
    
    Attributes
    ----------
    a1 : float
        the value of the angular acceleration of the first joint
    
    a2 : float
        the value of the angular acceleration of the second joint
    
    a3 : float
        the value of the angular acceleration of the third joint
    
    a4 : float
        the value of the angular acceleration of the fourth joint
        
    a5 : float
        the value of the angular acceleration of the fifth joint
        
    a6 : float
        the value of the angular acceleration of the sixth joint
        
    Methods
    -------
    to_numpy(self) -> np.ndarray
        converts a acceleration instance to a numpy array
    '''
    a1: float
    a2: float
    a3: float
    a4: float
    a5: float
    a6: float
    
    def __repr__(self) -> str:
        return f'({self.a1:.4f}, {self.a2:.4f}, {self.a3:.4f}, {self.a4:.4f}, {self.a5:.4f}, {self.a6:.4f})'
    
    def __add__(self, other):
        return Acceleration(self.a1+other.a1,
                            self.a2+other.a2,
                            self.a3+other.a3,
                            self.a4+other.a4,
                            self.a5+other.a5,
                            self.a6+other.a6)
        
    def __sub__(self, other):
        return Acceleration(self.a1-other.a1,
                            self.a2-other.a2,
                            self.a3-other.a3,
                            self.a4-other.a4,
                            self.a5-other.a5,
                            self.a6-other.a6)
    
    def __mul__(self, other):
        return Acceleration(self.a1*other,
                            self.a2*other,
                            self.a3*other,
                            self.a4*other,
                            self.a5*other,
                            self.a6*other)
    
    def __rmul__(self, other):
        return Acceleration(self.a1*other,
                            self.a2*other,
                            self.a3*other,
                            self.a4*other,
                            self.a5*other,
                            self.a6*other)
    
    def __truediv__(self, other):
        return Acceleration(self.a1/other,
                            self.a2/other,
                            self.a3/other,
                            self.a4/other,
                            self.a5/other,
                            self.a6/other)

    def __neg__(self):
        return Acceleration(-self.a1,
                            -self.a2,
                            -self.a3,
                            -self.a4,
                            -self.a5,
                            -self.a6)
    
    def to_numpy(self) -> np.ndarray:
        '''
        Converts the acceleration to a numpy array.
        
        Returns
        -------
        np.ndarray
            The acceleration as a numpy array.
        '''
        return np.array(self)
    