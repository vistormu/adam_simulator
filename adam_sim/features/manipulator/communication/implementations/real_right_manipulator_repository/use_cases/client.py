import pkg_resources
import yaml
import threading

from collections import deque
from paho.mqtt.client import Client, MQTTMessage
from vclog import Logger

from .......entities import Configuration

filename = pkg_resources.resource_filename('adam_sim', 'core/topics.yaml')
with open(filename, 'r') as file:
    data: dict = yaml.safe_load(file)
    configuration_topic_command: str = data['right_manipulator']['configuration']['command']
    configuration_topic_receive: str = data['right_manipulator']['configuration']['receive']


class MQTTClient:
    def __init__(self, host: str, port: int) -> None:
        # Logger
        self.logger: Logger = Logger('mqtt')

        self.host: str = host
        self.port: int = port

        # Client
        self.client = Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(host, port)

        threading.Thread(target=self.client.loop_forever).start()

        # Variables
        self.queue: deque = deque(maxlen=12)
        self.active: bool = True

    def on_connect(self, client, userdata, flags, rc):
        self.logger.info("Connected with result code "+str(rc))
        self.client.subscribe(configuration_topic_receive)

    def on_message(self, client, userdata, message: MQTTMessage):
        if message.topic == configuration_topic_receive:
            self.queue.append(Configuration.from_yaml(message.payload.decode()))

    def publish_configuration(self, configuration: Configuration) -> None:
        self.client.publish(configuration_topic_command, configuration.to_yaml())

    def get_configuration(self) -> Configuration:
        while len(self.queue) == 0 and self.active:
            pass

        configuration: Configuration = self.queue.pop()
        self.queue.clear()
        return configuration

    def close(self) -> None:
        self.active = False
        self.client.disconnect()
