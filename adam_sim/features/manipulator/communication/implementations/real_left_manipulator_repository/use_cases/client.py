import pkg_resources
import yaml
import numpy as np

from collections import deque
from paho.mqtt.client import Client, MQTTMessage
from vclog import Logger

from .......entities import Configuration

filename = pkg_resources.resource_filename('adam_sim', 'core/topics.yaml')
with open(filename, 'r') as file:
    data: dict = yaml.safe_load(file)
    configuration_topic_command: str = data['left_manipulator']['configuration']['command']
    configuration_topic_receive: str = data['left_manipulator']['configuration']['receive']
    velocity_topic_command: str = data['left_manipulator']['velocity']['command']
    velocity_topic_receive: str = data['left_manipulator']['velocity']['receive']


class MQTTClient:
    def __init__(self, host: str, port: int) -> None:
        # Logger
        self.logger: Logger = Logger('mqtt')

        self.host: str = host
        self.port: int = port

        # Client
        self.client = Client()
        self.client.connect(host, port)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.loop_start()

        # Variables
        self.queue: deque = deque()

    def on_connect(self, client, userdata, flags, rc):
        self.logger.info("Connected with result code "+str(rc))
        self.client.subscribe(configuration_topic_receive)

    def on_message(self, client, userdata, message: MQTTMessage):
        if message.topic == configuration_topic_receive:
            self.queue.append(Configuration.from_yaml(message.payload.decode()))
            self.client.loop_stop()

    def publish_configuration(self, configuration: Configuration) -> None:
        self.client.publish(configuration.to_yaml())

    def get_configuration(self) -> Configuration:
        self.client.loop_forever()
        configuration: Configuration = self.queue.popleft()
        # self.queue.clear()
        return configuration
