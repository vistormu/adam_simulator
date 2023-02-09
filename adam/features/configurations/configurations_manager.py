import pandas as pd

from ...core.entities import Configuration

DATA_PATH: str = 'adam/data/'


class ConfigurationsManager:
    @staticmethod
    def load(filename: str) -> list[Configuration]:
        if filename == 'test':
            filename = 'adam/features/configurations/data/configurations_test.csv'

        data_frame: pd.DataFrame = pd.read_csv(filename)

        configuration_list: list[Configuration] = [Configuration(*row) for row in data_frame.to_numpy()]

        return configuration_list

    @staticmethod
    def save(filename: str, configuration_list: list[Configuration]) -> None:
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
