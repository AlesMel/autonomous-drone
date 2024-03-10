from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.side_channel import (
    SideChannel,
    IncomingMessage,
    OutgoingMessage,
)
import numpy as np
import uuid

class AgentLogChannel(SideChannel):
    def __init__(self) -> None:
        super().__init__(uuid.UUID("58caf6d4-08f6-4996-8aba-72ed74670acd"))
        self.received_true_message = False  # Flag to indicate receipt of a True message

    def on_message_received(self, msg: IncomingMessage) -> None:
        print(msg.read_bool)
        try:
            received_bool = msg.read_bool()
            print(f"Received boolean: {received_bool}")
            self.received_true_message = received_bool
        except Exception as e:
            print(f"Error reading message: {e}")

    def send_string(self, data: str):
        msg = OutgoingMessage()
        msg.write_string(data)
        super().queue_message_to_send(msg=msg)
        
    def send_int(self, data: int, verbose = 0):
        if verbose == 1:
            print(f"Sending: {data}")
        msg = OutgoingMessage()
        msg.write_int32(i=data)
        super().queue_message_to_send(msg=msg)