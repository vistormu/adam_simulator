from dataclasses import dataclass

from ....core.entities.configuration import Configuration


@dataclass
class ConfigurationData:
    '''
    the ConfigurationData class is a dataclass that contains the configuration information of the left and right manipulator
    
    Attributes
    ----------
    left_manipulator : ~.entities.Configuration
        the configuration of the left manipulator
        
    right_manipulator : ~.entities.Configuration
        the configuration of the right manipulator
    '''
    left_manipulator: Configuration
    right_manipulator: Configuration
    