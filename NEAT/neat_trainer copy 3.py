from abc import ABC, abstractmethod
from mlagents_envs.environment import UnityEnvironment

class UnityGym():
    def __init__(
            self,
            unity_env: UnityEnvironment
            ):
        self._env = unity_env