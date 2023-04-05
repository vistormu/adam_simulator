from typing import NamedTuple

class Collision(NamedTuple):
    '''
    the Collision class is a NamedTuple containing all the information of a collision
    
    Attributes
    ----------
    collided : bool
        indicates if a collision has been produced
        
    self_collision : bool
        indicates is a self collision has been produced
        
    env_collision : bool
        indicates if a collision with the environment has been produced
        
    vector : list[bool]
        a list containing the collision information on every link
        
    shoulder : list[str]
        the list of elements that have collided with the shoulder
    
    upper_arm : list[str]
        the list of elements that have collided with the upper_arm
    
    forearm : list[str]
        the list of elements that have collided with the forearm
    
    wrist_1 : list[str]
        the list of elements that have collided with the wrist 1
    
    wrist_2 : list[str]
        the list of elements that have collided with the wrist 2
        
    wrist_3 : list[str]
        the list of elements that have collided with the wrist 3
    
    Methods
    -------
    empty(cls) -> ~.entities.Collision
        a class method that returns an empty instance of itself
    '''
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
        '''
        returns an empty instance of the class
        '''
        return cls(False, False, False, [False]*6, [], [], [], [], [], [])
    