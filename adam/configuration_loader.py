import pandas as pd

from .entities import Configuration

DATA_PATH: str = 'adam/data/'


class ConfigurationLoader:
    @staticmethod
    def load(filename: str) -> list[Configuration]:
        data_frame: pd.DataFrame = pd.read_csv(DATA_PATH+filename)

        number_of_rows: int = len(data_frame.index)

        configuration_list: list[Configuration] = []
        for i in range(number_of_rows):
            configuration: Configuration = Configuration(data_frame['theta_1'][i],
                                                         data_frame['theta_2'][i],
                                                         data_frame['theta_3'][i],
                                                         data_frame['theta_4'][i],
                                                         data_frame['theta_5'][i],
                                                         data_frame['theta_6'][i])

            configuration_list.append(configuration)

        return configuration_list
