import pickle
import numpy as np
import neat
import datetime
import os
from comunication_channel import AgentLogChannel

from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

NUM_RUNS = 1
MAX_GENS = 1000

engine_config_channel = EngineConfigurationChannel()
engine_config_channel.set_configuration_parameters(time_scale=5)
agent_count_channel = AgentLogChannel()

env_path = "../Builds/train-env/autonomous-drone.exe"
save_nn_destination = 'result/best.pkl'

env = UnityEnvironment(file_name=None, worker_id=0, no_graphics=True, side_channels=[engine_config_channel, agent_count_channel])
env.reset()

num_actions = 4 # 4
num_inputs = 18 # 14
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

def create_folder(file_name_prefix):
    directory = os.path.dirname(f"{file_name_prefix}")
    if not os.path.exists(directory):
        os.makedirs(directory)

def set_agents_and_double_reset(num_agents: int):
    agent_count_channel.send_int(data=num_agents) 
    env.reset()
    env.reset() 

def get_observation_for_agent(agent: int, observations):
    for observation in observations:
        key = int(observation.split("=")[2])
        if key == agent:
            return observations[observation]
        
def map_agent_ids(decision_steps):
    """
    Map agent ids between NEAT and UNITY.

    Args:
        decision_steps: An iterable containing decision steps.

    Returns:
        A tuple of two dictionaries: (unity_to_neat_map, neat_to_unity_map)
    """
    unity_to_neat_map = {}
    neat_to_unity_map = {}
    id_count = 0
    for step in decision_steps:
        unity_to_neat_map[step] = id_count
        neat_to_unity_map[id_count] = step
        id_count += 1
    return unity_to_neat_map, neat_to_unity_map

def create_policies(genomes, cfg):
    policies = []
    for _, g in genomes:
        g.fitness = 0
        policy = neat.nn.FeedForwardNetwork.create(g, cfg)
        policies.append(policy)
    return policies

import random
def eval_genomes(genomes, cfg):
    policies = create_policies(genomes, cfg)
    set_agents_and_double_reset(len(policies))
    decision_steps, terminal_steps = env.get_steps(behavior_name)
    agent_count = len(decision_steps.agent_id)

    unity_to_neat_map, neat_to_unity_map = map_agent_ids(decision_steps)

    done = False  # Vectorized initialization
    removed_agents = []

    episode_rewards = [0] * agent_count
    print(f"Agent count: {agent_count}")
    env.reset()
    while not done:
        for agent in decision_steps:
            if agent not in removed_agents:
                nn_input =  np.asarray(decision_steps[agent].obs[:])
                actions = policies[unity_to_neat_map[agent]].activate(nn_input[0])
                continous_actions = np.asarray([actions])
                action_tuple = ActionTuple(discrete=None, continuous=continous_actions)
                env.set_action_for_agent(behavior_name=behavior_name, 
                                        agent_id=agent, 
                                        action=action_tuple)
        env.step()
        decision_steps, terminal_steps = env.get_steps(behavior_name)
        for agent in range(agent_count):
            if agent not in removed_agents:
                local_agent = neat_to_unity_map[agent]
                if local_agent in terminal_steps:
                    episode_rewards[agent] += terminal_steps[local_agent].reward
                    removed_agents.append(agent)
                elif local_agent in decision_steps:
                    episode_rewards[agent] += decision_steps[local_agent].reward
                 

        if len(removed_agents) >= agent_count:
            print(".") 
            done = True
    for i, (_, genome) in enumerate(genomes):
        genome.fitness = episode_rewards[i]
    env.reset()

def run(config_file, run, datte):
    print(f"Running {run}")
    file_name_prefix = f"checkpoints/{datte}/run-{run}/"
    create_folder(file_name_prefix=file_name_prefix)

    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)
    
    pop = neat.Population(config)
    pop.add_reporter(neat.Checkpointer(generation_interval=5, time_interval_seconds=100000, filename_prefix=file_name_prefix))
    stats = neat.StatisticsReporter()
    pop.add_reporter(stats)
    pop.add_reporter(neat.TBReporter(False, 0, run, datte))
    #pop.add_reporter(neat.StdOutReporter(True))
    best = pop.run(eval_genomes, MAX_GENS)
    # Display the winning genome.
    print('\nBest genome:\n{!s}'.format(best))
    print("Finished running!")
    
    # Save best genome
    with open(f'logs/{datte}/{run}/best.pkl', 'wb') as f:
        pickle.dump(best, f)

if __name__ == "__main__":
    config_path = 'NEAT\conv_config'
    datte = datetime.datetime.now().strftime("%d-%m-%Y--%H_%M")
    # pre create folders for checkpointer
    for r in range(NUM_RUNS):
        run(config_path, r, datte)
    env.close()
