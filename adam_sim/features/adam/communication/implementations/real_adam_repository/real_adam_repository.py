import time
from paho.mqtt.client import Client

from ....repository import AdamRepository


class RealAdamRepository(AdamRepository):
    def init(self, host: str, port: int, rate: int) -> None:
        self.client = Client()
        self.client.connect(host, port)

        self.count: int = 0
        self.rate: int = rate

    def step(self) -> None:
        self.client.publish("mqtt/adam", self.count)
        self.count += 1

        time.sleep(1.0 / self.rate)
