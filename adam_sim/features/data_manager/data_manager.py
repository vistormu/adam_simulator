import pandas as pd
import pkg_resources


from ...entities import Configuration, Point


class DataManager:
    '''
    the ConfigurationsManager class manages the external communication with files

    Methods
    -------
    load_configurations:
        loads a list of configurations

    save_configurations:
        saves a list of configurations

    load_points:
        loads a list of points

    save_points:
        saves a list of points
    '''
    @staticmethod
    def load_configurations(filename: str) -> list[Configuration]:
        '''
        loads a list of configurations

        Parameters
        ----------
        filename : str
            the path to the file to load

        Returns
        -------
        out : list[~.entities.Configurations]
            a list of configurations
        '''
        if filename == 'test':
            filename = pkg_resources.resource_filename('adam_sim', 'features/data_manager/data/configurations_test.csv')

        return [Configuration.from_numpy(row) for row in pd.read_csv(filename).to_numpy()]

    @staticmethod
    def save_configurations(filename: str, configuration_list: list[Configuration]) -> None:
        '''
        saves a list of configurations

        Parameters
        ----------
        filename : str
            the path to the file to save

        configuration_list : list[~.entities.Configurations]
            a list of configurations
        '''
        data: dict = {
            'theta_1': [configuration.q1 for configuration in configuration_list],
            'theta_2': [configuration.q2 for configuration in configuration_list],
            'theta_3': [configuration.q3 for configuration in configuration_list],
            'theta_4': [configuration.q4 for configuration in configuration_list],
            'theta_5': [configuration.q5 for configuration in configuration_list],
            'theta_6': [configuration.q6 for configuration in configuration_list],
        }

        pd.DataFrame(data).to_csv(filename, index=False)

    @staticmethod
    def load_points(filename: str) -> list[Point]:
        '''
        loads a list of points

        Parameters
        ----------
        filename : str
            the path to the file to load

        Returns
        -------
        out : list[~.entities.Point]
            a list of points
        '''
        if filename == 'test':
            filename = pkg_resources.resource_filename('adam_sim', 'features/data_manager/data/end_effector_positions_test.csv')

        return [Point.from_numpy(row) for row in pd.read_csv(filename).to_numpy()]

    @staticmethod
    def save_points(filename: str, end_effector_positions: list[Point]) -> None:
        '''
        saves a list of points

        Parameters
        ----------
        filename : str
            the path to the file to save

        end_effector_positions : list[~.entities.Point]
            a list of points
        '''
        data: dict = {
            'x': [end_effector_position.x for end_effector_position in end_effector_positions],
            'y': [end_effector_position.y for end_effector_position in end_effector_positions],
            'z': [end_effector_position.z for end_effector_position in end_effector_positions],
        }

        pd.DataFrame(data).to_csv(filename, index=False)
