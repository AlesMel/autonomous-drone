import pickle
import sys
import numpy as np
import time
import atexit   
import neat
# import visualize
import math
import csv
import os
import datetime

#from game_config import game_config
# MLAGENTS stuff
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

is_training = False
env_path = "./Builds/test-env/autonomous-drone.exe"
save_nn_destination = 'NEAT/result/best.pkl'

engine_config_channel = EngineConfigurationChannel()
engine_config_channel.set_configuration_parameters(time_scale=1)

env = UnityEnvironment(file_name=env_path, seed=0, no_graphics=False, side_channels=[engine_config_channel])
env.reset()

num_actions = 4
num_inputs = 14
out_mult = 1

behavior_specs = env.behavior_specs
print(f"Behaviour specs {behavior_specs}")
behavior_name = list(behavior_specs)[0]
spec = env.behavior_specs[behavior_name]

print(f"Name of the behavior : {behavior_name}")
print("Number of observations : ", len(spec.observation_specs)) # vector if 1

# Is the Action continuous or multi-discrete ?
if spec.action_spec.continuous_size > 0:
  print(f"There are {spec.action_spec.continuous_size} continuous actions")
if spec.action_spec.is_discrete():
  print(f"There are {spec.action_spec.discrete_size} discrete actions")

def create_policies(genomes, cfg):
    policies = []
    for _, g in genomes:
        g.fitness = 0
        policy = neat.nn.FeedForwardNetwork.create(g, cfg)
        policies.append(policy)
    return policies

def eval_agent(genome, cfg):
    for _ in range(10):
        env.reset()
        decision_steps, terminal_steps = env.get_steps(behavior_name)
        agent_count = len(decision_steps.agent_id)
        done = [False for i in range(agent_count)]
        policy = neat.nn.FeedForwardNetwork.create(genome, cfg)
        policies = [policy for agent in range(agent_count)]
        #print(f"Agent count: {agent_count}")
        while not all(done):
            for agent in decision_steps:
                if done[agent] is False:
                    #print(f"Agent requesting decision step: {agent}")
                    nn_input = np.asarray(decision_steps[agent].obs[:])
                    #print(nn_input[0])
                    actions = policies[agent].activate(nn_input[0])
                    continous_actions = np.asarray([actions])
                    action_tuple = ActionTuple(discrete=None, continuous=continous_actions)
                    env.set_action_for_agent(behavior_name=behavior_name, 
                                            agent_id=agent, 
                                            action=action_tuple)
                
            env.step()
            decision_steps, terminal_steps = env.get_steps(behavior_name)

            for agent in terminal_steps: # The agent terminated its episode
                if done[agent] is False:
                    done[agent] = True


if __name__ == "__main__":
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config')
    with open(save_nn_destination, "rb") as f:
        genome = pickle.load(f)
        print(genome)
    print(genome.fitness)
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                        neat.DefaultSpeciesSet, neat.DefaultStagnation,
                        config_path)

    eval_agent(genome, config)
