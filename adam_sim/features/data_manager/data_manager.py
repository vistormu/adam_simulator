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

    load_end_effector_positions:
        loads a list of end effector positions

    save_end_effector_positions:
        saves a list of end effector positions
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

        data_frame: pd.DataFrame = pd.read_csv(filename)

        configuration_list: list[Configuration] = [Configuration(*row) for row in data_frame.to_numpy()]

        return configuration_list

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

        data_frame: pd.DataFrame = pd.DataFrame(data)

        data_frame.to_csv(filename, index=False)

    @staticmethod
    def load_end_effector_positions(filename: str) -> list[Point]:
        '''
        loads a list of end effector positions

        Parameters
        ----------
        filename : str
            the path to the file to load

        Returns
        -------
        out : list[~.entities.Point]
            a list of end effector positions
        '''
        if filename == 'test':
            filename = pkg_resources.resource_filename('adam_sim', 'features/data_manager/data/end_effector_positions_test.csv')

        data_frame: pd.DataFrame = pd.read_csv(filename)

        end_effector_positions: list[Point] = [Point(*row) for row in data_frame.to_numpy()]

        return end_effector_positions

    @staticmethod
    def save_end_effector_positions(filename: str, end_effector_positions: list[Point]) -> None:
        '''
        saves a list of end effector positions

        Parameters
        ----------
        filename : str
            the path to the file to save

        end_effector_positions : list[~.entities.Point]
            a list of end effector positions
        '''
        data: dict = {
            'x': [end_effector_position.x for end_effector_position in end_effector_positions],
            'y': [end_effector_position.y for end_effector_position in end_effector_positions],
            'z': [end_effector_position.z for end_effector_position in end_effector_positions],
        }

        data_frame: pd.DataFrame = pd.DataFrame(data)

        data_frame.to_csv(filename, index=False)
