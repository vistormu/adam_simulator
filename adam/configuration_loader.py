import pandas as pd

from .entities import Configuration

DATA_PATH: str = 'adam/data/'


class ConfigurationLoader:
    @staticmethod
    def load(filename: str) -> list[Configuration]:
        data_frame: pd.DataFrame = pd.read_csv(DATA_PATH+filename)

        configuration_list: list[Configuration] = [Configuration(*row) for row in data_frame.to_numpy()]

        return configuration_list
