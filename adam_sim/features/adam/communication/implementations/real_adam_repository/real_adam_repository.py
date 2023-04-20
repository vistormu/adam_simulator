import time
import pkg_resources
import yaml
from paho.mqtt.client import Client

from ....repository import AdamRepository


class RealAdamRepository(AdamRepository):
    def init(self, host: str, port: int, rate: int) -> None:
        # Connect to mosquitto
        self.client: Client = Client()
        self.client.connect(host, port)

        # Variables
        filename = pkg_resources.resource_filename('adam_sim', 'core/topics.yaml')
        with open(filename, 'r') as file:
            data: dict = yaml.safe_load(file)
            self.topic: str = data['adam']

        self.client.publish(self.topic, False)
        time.sleep(1)
        self.client.publish(self.topic, False)

        # Variable initialization
        self.rate: int = rate

    def step(self) -> None:
        self.client.publish(self.topic, True)

        time.sleep(1.0/self.rate)

    def close(self) -> None:
        self.client.disconnect()
