import numpy as np
import yaml

from typing import NamedTuple


class Configuration(NamedTuple):
    '''
    the Configuration class is a NamedTuple that contains the values of the joint angles. Unit: rad

    Attributes
    ----------
    q1 : float
        the value of the first joint

    q2 : float
        the value of the second joint

    q3 : float
        the value of the third joint

    q4 : float
        the value of the fourth joint

    q5 : float
        the value of the fifth joint

    q6 : float
        the value of the sixth joint

    Methods
    -------
    to_degrees(self) -> ~.entities.Configuration
        converts a configuration instance from radians to degrees

    to_radians(self) -> ~.entities.Configuration
        converts a configuration instance from degrees to radians

    round(self, ndigits: int) -> ~.entities.Configuration
        rounds a configuration instance to a given number of digits

    to_numpy(self) -> np.ndarray
        converts a configuration instance to a numpy array

    to_list(self) -> list[float]
        converts a configuration instance to a list

    to_yaml(self) -> str
        converts a configuration instance to a yaml string

    from_yaml(self, yaml_string: str) -> ~.entities.Configuration
        converts a yaml string to a configuration instance

    from_numpy(self, array: np.ndarray) -> ~.entities.Configuration
        converts a numpy array to a configuration instance

    from_list(self, list: list[float]) -> ~.entities.Configuration
        converts a list to a configuration instance


    Notes
    -----
    the Configuration class supports addition, subtraction, division and multiplication
    '''
    q1: float
    q2: float
    q3: float
    q4: float
    q5: float
    q6: float

    def __repr__(self) -> str:
        return f'({self.q1:.4f}, {self.q2:.4f}, {self.q3:.4f}, {self.q4:.4f}, {self.q5:.4f}, {self.q6:.4f})'

    def __add__(self, other: 'Configuration') -> 'Configuration':
        return Configuration(self.q1+other.q1, self.q2+other.q2, self.q3+other.q3, self.q4+other.q4, self.q5+other.q5, self.q6+other.q6)

    def __sub__(self, other: 'Configuration') -> 'Configuration':
        return Configuration(self.q1-other.q1, self.q2-other.q2, self.q3-other.q3, self.q4-other.q4, self.q5-other.q5, self.q6-other.q6)

    def __mul__(self, scalar: float) -> 'Configuration':
        return Configuration(self.q1*scalar, self.q2*scalar, self.q3*scalar, self.q4*scalar, self.q5*scalar, self.q6*scalar)

    def __rmul__(self, scalar: float) -> 'Configuration':
        return Configuration(self.q1*scalar, self.q2*scalar, self.q3*scalar, self.q4*scalar, self.q5*scalar, self.q6*scalar)

    def __truediv__(self, scalar: float) -> 'Configuration':
        return Configuration(self.q1/scalar,  self.q2/scalar,  self.q3/scalar, self.q4/scalar, self.q5/scalar, self.q6/scalar)

    def __neg__(self) -> 'Configuration':
        return Configuration(-self.q1, -self.q2, -self.q3, -self.q4, -self.q5, -self.q6)

    def __eq__(self, other: 'Configuration') -> bool:
        return self.q1 == other.q1 and self.q2 == other.q2 and self.q3 == other.q3 and self.q4 == other.q4 and self.q5 == other.q5 and self.q6 == other.q6

    def __ne__(self, other: 'Configuration') -> bool:
        return not self.__eq__(other)

    def __round__(self, n: int) -> 'Configuration':
        return Configuration(round(self.q1, n), round(self.q2, n), round(self.q3, n), round(self.q4, n), round(self.q5, n), round(self.q6, n))

    def to_degrees(self) -> 'Configuration':
        '''
        converts the instance from radians to degrees

        Returns
        -------
        ~.entities.Configuration
            the instance in degrees
        '''
        return self*180/np.pi

    def to_radians(self) -> 'Configuration':
        '''
        converts the instance from degrees to radians

        Returns
        -------
        ~.entities.Configuration
            the instance in radians
        '''
        return self*np.pi/180

    def round(self, n: int = 4) -> 'Configuration':
        '''
        rounds the instance to n decimal places

        Parameters
        ----------
        n : int, optional
            the number of decimal places to round to. Default: 4

        Returns
        -------
        ~.entities.Configuration
            the rounded instance
        '''
        return Configuration(*np.round(self, n))

    def to_numpy(self) -> np.ndarray:
        '''
        converts the instance to a numpy array

        Returns
        -------
        np.ndarray
            the numpy array representation of the instance
        '''
        return np.array(self)

    @classmethod
    def from_numpy(cls, array: np.ndarray) -> 'Configuration':
        '''
        converts a numpy array to a Configuration instance

        Parameters
        ----------
        array : np.ndarray
            the numpy array to convert

        Returns
        -------
        ~.entities.Configuration
            the Configuration instance
        '''
        return Configuration(*array)

    def to_list(self) -> list:
        '''
        converts the instance to a list

        Returns
        -------
        list
            the list representation of the instance
        '''
        return list(self)

    @classmethod
    def from_list(cls, list: list) -> 'Configuration':
        '''
        converts a list to a Configuration instance

        Parameters
        ----------
        list : list
            the list to convert

        Returns
        -------
        ~.entities.Configuration
            the Configuration instance
        '''
        return Configuration(*list)

    def to_yaml(self) -> str:
        '''
        converts the instance to a yaml string

        Returns
        -------
        str
            the yaml string representation of the instance
        '''
        return yaml.dump([str(q) for q in self])

    @classmethod
    def from_yaml(cls, yaml_string: str) -> 'Configuration':
        '''
        converts a yaml string to a Configuration instance

        Parameters
        ----------
        yaml_string : str
            the yaml string to convert

        Returns
        -------
        ~.entities.Configuration
            the Configuration instance
        '''
        return Configuration.from_numpy(np.array(yaml.safe_load(yaml_string)).astype(float))
