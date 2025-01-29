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
from comunication_channel import AgentLogChannel
#from game_config import game_config
# MLAGENTS stuff
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

env_path = "./Builds/test-side-channel-env/autonomous-drone.exe"
save_nn_destination = 'NEAT/result/best.pkl'

engine_config_channel = EngineConfigurationChannel()
engine_config_channel.set_configuration_parameters(time_scale=10)
agent_count_channel = AgentLogChannel()

env = UnityEnvironment(file_name=None, no_graphics=True, side_channels=[engine_config_channel, agent_count_channel])
#env = UnityEnvironment(side_channels=[agent_count_channel])

env.reset()

num_actions = 4 # 4
num_inputs = 13 # 14
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

def open_env(enviroment):
    #enviroment = UnityEnvironment(file_name=env_path, no_graphics=True, side_channels=[engine_config_channel, agent_count_channel])
    enviroment = UnityEnvironment(side_channels=[agent_count_channel])
    enviroment.reset()
    return enviroment

def create_policies(genomes, cfg):
    policies = []
    for _, g in genomes:
        policy = neat.nn.FeedForwardNetwork.create(g, cfg)
        policies.append(policy)
        g.fitness = 0
    return policies

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

def eval_genomes(genomes, cfg):
    policies = create_policies(genomes, cfg)
    agent_count_channel.send_int(data=len(policies))   
    decision_steps, terminal_steps = env.get_steps(behavior_name)
    agent_count = len(decision_steps.agent_id)

    # this is here because when dynamically removing agents, the env.reset() and decision_steps don't cautgh up nicely (probably a timing issue)
    while agent_count != len(policies):
        print(f"{agent_count} vs {len(policies)}")
        env.reset()
        decision_steps, terminal_steps = env.get_steps(behavior_name)
        agent_count = len(decision_steps.agent_id)
        
    unity_to_neat_map, neat_to_unity_map = map_agent_ids(decision_steps)

    done = False  # Vectorized initialization
    removed_agents = []

    episode_rewards = [0] * agent_count
    print(f"Agent count: {agent_count}")

    while not done:
        for agent in decision_steps:
            nn_input = np.asarray(decision_steps[agent].obs[:])
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
                 
            # genomes[agent][1].fitness += reward

        if len(removed_agents) >= agent_count:
            print(".") 
            done = True
    for i, (_, genome) in enumerate(genomes):
        genome.fitness = episode_rewards[i]
    env.reset()
    print("\nFinished generation")

def run(config_file, run):
    print(f"Running {run}")
    max_generations = 1000

    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)
    
    pop = neat.Population(config)

    stats = neat.StatisticsReporter()

    pop.add_reporter(stats)
    #pop.add_reporter(neat.Checkpointer(generation_interval=25, time_interval_seconds=1200, filename_prefix='NEAT/checkpoints/NEAT-checkpoint-'))
    pop.add_reporter(neat.TBReporter(True, 0, run, datte))
    #pop.add_reporter(neat.StdOutReporter(True))
    env.reset()
    best = pop.run(eval_genomes, max_generations)
    # Display the winning genome.
    print('\nBest genome:\n{!s}'.format(best))
    print("Finished running!")
    
    # Save best genome
    with open(f'logs/{datte}/{run}/best.pkl', 'wb') as f:
        pickle.dump(best, f)

if __name__ == "__main__":
    #atexit.register(exit_handler)
    datte = datetime.datetime.now().strftime("%d-%m-%Y--%H_%M")
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config')
    runs = 3
    for r in range(runs):
        run(config_path, r)
        # if r < runs:
        #     env.close()
        #     env = open_env(env)
    print("Closing environment!")
    env.close()
