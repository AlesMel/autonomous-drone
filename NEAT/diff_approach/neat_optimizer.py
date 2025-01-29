from mlagents.trainers.settings import (
    TrainerSettings,
    OnPolicyHyperparamSettings,
    ScheduleType,
)

from mlagents.trainers.optimizer.torch_optimizer import TorchOptimizer
from mlagents.trainers.policy.torch_policy import TorchPolicy

import pickle
import sys
import numpy as np
import time
import atexit
import neat
import visualize
import math
import csv

@attr.s(auto_attribs=True)
class NEATSettings(OnPolicyHyperparamSettings):
    atexit.register(exit_handler)
    config_path = "NEAT/config_ctrnn"
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation)
    

class TorchPPOOptimizer(TorchOptimizer):
    def __init__(self, policy: TorchPolicy, trainer_settings: TrainerSettings):
        super().__init__(policy, trainer_settings)
