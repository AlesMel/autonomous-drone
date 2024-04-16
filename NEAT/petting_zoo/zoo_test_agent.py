from mlagents_envs.environment import UnityEnvironment  # Import Unity environment
from mlagents_envs.envs.unity_aec_env import UnityAECEnv
from mlagents_envs.envs.unity_parallel_env import UnityParallelEnv
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from comunication_channel import AgentLogChannel
import neat
import os
import datetime
import pickle
import numpy as np

engine_config_channel = EngineConfigurationChannel()
engine_config_channel.set_configuration_parameters(time_scale=1)
agent_count_channel = AgentLogChannel()

env_path = "./Builds/train-env/autonomous-drone.exe"
save_nn_destination = 'NEAT/result/best.pkl'

env = UnityEnvironment(file_name=None, worker_id=0, no_graphics=False, side_channels=[engine_config_channel, agent_count_channel])
env = UnityAECEnv(env)
env.reset()
num_agents = len(env.possible_agents)

num_actions = env.action_space(env.possible_agents[0]).shape[0]
print(f"There is total of {num_actions} actions in enviroment")
num_inputs = env.observation_space(env.possible_agents[0]).shape[0]
print(f"There is total of {num_inputs} inputs in enviroment")

MAX_STEPS = 1200
NUM_TRIES = 3

def map_agents():
    map = {}
    current_index = 0
    for agent in env.agents:
        map[int(agent.split("=")[2])] = current_index
        current_index += 1
    return map

def set_agents_and_double_reset(num_agents: int):
    agent_count_channel.send_int(data=num_agents) 
    env.reset()
    env.reset()

def create_policies(genomes, cfg):
    policies = []
    for _, g in genomes:
        policy = neat.nn.FeedForwardNetwork.create(g, cfg)
        policies.append(policy)
        g.fitness = 0
    return policies

def eval_genome(genome, cfg):
    policies = [neat.nn.FeedForwardNetwork.create(genome, cfg)] * env.num_agents
    map = map_agents()
    for _ in range(NUM_TRIES):
        rewards = [0] * env.num_agents
        for agent in env.agent_iter(env.num_agents * MAX_STEPS):
            current_agent = int(agent.split("=")[2])
            obs, reward, done, info = env.last(observe=True)
            if done:
                action = None
            else:
                action = np.asarray(policies[map[current_agent]].activate(obs))
            rewards[map[current_agent]] += reward
            env.step(action)
        env.reset()
        print(rewards)
if __name__ == "__main__":
    datte = datetime.datetime.now().strftime("%d-%m-%Y--%H_%M")
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'test_config')
    with open(save_nn_destination, "rb") as f:
        genome = pickle.load(f)
        print(genome)
    print(genome.fitness)
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                        neat.DefaultSpeciesSet, neat.DefaultStagnation,
                        config_path)

    eval_genome(genome, config)
    env.close()