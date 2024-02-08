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
        super().__init__(uuid.UUID("621f0a70-4f87-11ea-a6bf-784f4387d1f7"))
    def on_message_received(self, msg: IncomingMessage) -> None:
        print(msg.read_string())
    def send_string(self, data: str):
        msg = OutgoingMessage()
        msg.write_string(data)
        super().queue_message_to_send(msg=msg)
    def send_int(self, data: int):
        msg = OutgoingMessage()
        msg.write_int32(i=data)
        super().queue_message_to_send(msg=msg)