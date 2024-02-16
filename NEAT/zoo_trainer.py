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
engine_config_channel.set_configuration_parameters(time_scale=10)
agent_count_channel = AgentLogChannel()

env_path = "./Builds/train-env/autonomous-drone.exe"

env = UnityEnvironment(file_name=env_path, worker_id=0, no_graphics=True, side_channels=[engine_config_channel, agent_count_channel])
env = UnityAECEnv(env)
env.reset()
num_agents = len(env.possible_agents)

num_actions = env.action_space(env.possible_agents[0]).shape[0]
print(f"There is total of {num_actions} actions in enviroment")
num_inputs = env.observation_space(env.possible_agents[0]).shape[0]
print(f"There is total of {num_inputs} inputs in enviroment")

MAX_STEPS = 1500
NUM_RUNS = 5
MAX_GENS = 2500

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

def eval_genomes(genomes, cfg):
    policies = create_policies(genomes, cfg)
    target_agents = len(policies)
    set_agents_and_double_reset(num_agents = target_agents)
    rewards = [0] * target_agents
    map = map_agents()
    for agent in env.agent_iter(env.num_agents * MAX_STEPS):
        current_agent = int(agent.split("=")[2])
        obs, reward, done, info = env.last(observe=True)
        if done:
            action = None
        else:
            action = np.asarray(policies[map[current_agent]].activate(obs))
        rewards[map[current_agent]] += reward
        env.step(action)
    for i, (_, genome) in enumerate(genomes):
        if rewards[i] > 4000:
            print("Bullshit!")  
        genome.fitness = rewards[i]
    env.reset()
    print("\nFinished generation")

def run(config_file, run, datte):
    print(f"Running {run}")

    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)
    
    pop = neat.Population(config)

    stats = neat.StatisticsReporter()

    pop.add_reporter(stats)
    #pop.add_reporter(neat.Checkpointer(generation_interval=25, time_interval_seconds=1200, filename_prefix='NEAT/checkpoints/NEAT-checkpoint-'))
    pop.add_reporter(neat.TBReporter(False, 0, run, datte))
    #pop.add_reporter(neat.StdOutReporter(True))
    env.reset()
    best = pop.run(eval_genomes, MAX_GENS)
    # Display the winning genome.
    print('\nBest genome:\n{!s}'.format(best))
    print("Finished running!")
    
    # Save best genome
    with open(f'logs/{datte}/{run}/best.pkl', 'wb') as f:
        pickle.dump(best, f)

if __name__ == "__main__":
    datte = datetime.datetime.now().strftime("%d-%m-%Y--%H_%M")
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'test_config')
    for r in range(NUM_RUNS):
        run(config_path, r, datte)
    env.close()