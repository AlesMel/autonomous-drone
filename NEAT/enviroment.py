from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

# class Enviroment:
#     def __init__(self, env_path, is_training):
#         self.env = self.__create_enviroment(env_path=env_path, is_training=is_training)
def __create_enviroment(self, env_path, is_training):
    channel = EngineConfigurationChannel()
    if is_training:
        channel.set_configuration_parameters(time_scale=10)
    env = UnityEnvironment(env_path, seed=42, no_graphics=is_training, side_channels=[channel])
    return env        
