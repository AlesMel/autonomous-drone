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

#from game_config import game_config
# MLAGENTS stuff
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

# Params
is_training = True

env_path = "./Builds/autonomous-drone.exe"
test_env_path = "./Builds/single-agent-env/autonomous-drone.exe"
explorer_env_path = "./Builds/explorer/MappingDrone.exe"
save_nn_destination = 'NEAT/result/best.pkl'
engine_config_channel = EngineConfigurationChannel()
engine_config_channel.set_configuration_parameters(width=2048 , height=1080, quality_level=1, time_scale=1)

if is_training:
    env = UnityEnvironment(file_name=env_path, seed=0, no_graphics=False, side_channels=[engine_config_channel])
else:
    env = UnityEnvironment(file_name=test_env_path, seed=0)

env.reset()

num_actions = 4
num_inputs = 17

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
        policy = neat.nn.RecurrentNetwork.create(g, cfg)
        policies.append(policy)
    return policies

def process_agent_actions(agent_count, neat_to_unity_map, decision_steps, policies, nn_input):
    """
    Process actions for each agent based on their policies and neural network inputs.

    Args:
        agent_count: The total number of agents.
        neat_to_unity_map: A mapping from local agent indices to agent IDs.
        decision_steps: A collection containing the decision steps for each agent.
        policies: A list of policy objects corresponding to each agent.
        nn_input: Neural network input for each agent.

    Returns:
        Updated actions array for each agent.
    """
    #actions = np.ones(shape=(agent_count, num_inputs))
    actions = np.zeros(shape=(agent_count, len(policies[0].activate(nn_input[0]))))  # Assuming actions is a NumPy array
    for agent in range(agent_count):
        if neat_to_unity_map[agent] in decision_steps:
            actions[agent] = policies[agent].activate(nn_input[agent])
    return actions

def process_agent_inputs(agent_count, neat_to_unity_map, decision_steps):
    """
    Process inputs for each agent based on decision steps.

    Args:
        agent_count: The total number of agents.
        neat_to_unity_map: A mapping from local agent indices to agent IDs.
        decision_steps: A collection containing the decision steps for each agent.

    Returns:
        Updated neural network input array for each agent.
    """
    nn_input = np.zeros(shape=(agent_count, num_inputs)) 
    for agent in range(agent_count):
        if neat_to_unity_map[agent] in decision_steps:
            nn_input[agent] = np.asarray(decision_steps[neat_to_unity_map[agent]].obs[:])
            if agent == 0 and not is_training:
                print(f"Input for Agent{agent}: ", nn_input[agent])
    return nn_input

def process_and_set_agent_actions(agent_count, neat_to_unity_map, decision_steps, policies, nn_input):
    """
    Process agent actions based on policies and neural network input, 
    and set these actions in the environment.

    Args:
        agent_count: The total number of agents.
        local_to_agent_map: A mapping from local agent indices to agent IDs.
        decision_steps: A collection containing the decision steps for each agent.
        policies: A list of policy objects for each agent.
        nn_input: Neural network input for each agent.
        env: The environment object.
        behavior_name: The name of the behavior.

    Returns:
        Time spent in activating policies.
    """
    actions = np.zeros(shape=(agent_count, len(policies[0].activate(nn_input[0]))))  # Adjust shape as necessary
    actions = np.ones(shape=(agent_count, num_actions))

    for agent in range(agent_count):  
        if neat_to_unity_map[agent] in decision_steps:
            try:
                actions[agent] = policies[agent].activate(nn_input[agent])
            except:
                print("Exception occured!")
                print(f"Agent count: {agent_count}, policies length: {len(policies)}")
            continuous_actions = np.array([actions[agent, :]])
            #if agent == 0:
             # print(f"Action for Agent{agent}: ", continuous_actions)
            action_tuple = ActionTuple(discrete=None, continuous=continuous_actions)
            env.set_action_for_agent(behavior_name=behavior_name, 
                                     agent_id=neat_to_unity_map[agent], 
                                     action=action_tuple)

def collect_and_update_rewards(agent_count, neat_to_unity_map, terminal_steps, decision_steps, genomes):
    """
    Collect rewards for each agent and update the fitness of each genome.

    Args:
        agent_count: The total number of agents.
        neat_to_unity_map: A mapping from NEAT agent indices to Unity agent IDs.
        terminal_steps: A collection containing the terminal steps for each agent.
        decision_steps: A collection containing the decision steps for each agent.
        genomes: A list of genomes corresponding to each agent.
    """
    for agent in range(agent_count):
        if agent in neat_to_unity_map:
            local_agent = neat_to_unity_map[agent]
            reward = 0

            if local_agent in terminal_steps:
                reward += terminal_steps[local_agent].reward
            elif local_agent in decision_steps:
                reward += decision_steps[local_agent].reward
            genomes[agent][1].fitness += reward
    return genomes

def eval_agent(genomes, cfg):
    '''
    :param genomes: All the genomes in the current generation.
    :param cfg: Configuration file
    :return: Best genome from generation.
    '''
    decision_steps, terminal_steps = env.get_steps(behavior_name)
    agent_count = len(decision_steps.agent_id)

    # Map agent ids between NEAT and UNITY
    unity_to_neat_map, neat_to_unity_map = map_agent_ids(decision_steps)
    policies = create_policies(genomes, cfg)
    
    print(f"Policies length {len(policies)}")
    print(f"Population size (GENOMES): {len(genomes)}")
    print(f"Agent count: {agent_count}")

    done = [False if i < len(policies) else True for i in range(agent_count)]

    while not all(done):
        # Collect observations
        nn_input = process_agent_inputs(agent_count, neat_to_unity_map, decision_steps)
        process_and_set_agent_actions(agent_count, neat_to_unity_map, decision_steps, policies, nn_input)
        env.step()
        decision_steps, terminal_steps = env.get_steps(behavior_name)
        # Terminal agents
        for agent in terminal_steps:
            done[neat_to_unity_map[agent]] = True
        # Collect reward
        genomes = collect_and_update_rewards(agent_count, neat_to_unity_map, terminal_steps, decision_steps, genomes)

    print("--- [All agents are terminal!] ---")
    env.reset()
# TODO: do this on single agent env
def sim_agent(genome, cfg):
    decision_steps, terminal_steps = env.get_steps(behavior_name)
    unity_to_neat_map, neat_to_unity_map = map_agent_ids(decision_steps)
    for gen in range(100):
        decision_steps, terminal_steps = env.get_steps(behavior_name)
        policy = neat.nn.RecurrentNetwork.create(genome, cfg)
        done = False  # For the tracked_agent
        agent_count = len(decision_steps.agent_id)
        agent_id = neat_to_unity_map[0]
        while not done:
            nn_input = np.concatenate((decision_steps[agent_id].obs[:])) 
            if len(decision_steps) > 0:  # More steps to take?
                action = policy.activate(nn_input)  # FPass for purple action
                # Check if agent requests action:
                continuous_actions = np.array([action[:]])
                print(f"Action for Agent{agent_id}: ", action)
                action_tuple = ActionTuple(discrete=None, continuous=continuous_actions)
                env.set_action_for_agent(behavior_name=behavior_name, agent_id=agent_id, action=action_tuple)
            
            env.step()
            decision_steps, terminal_steps = env.get_steps(behavior_name)
            print(f"Terminal steps {terminal_steps.agent_id}")
            if agent_id in terminal_steps:
                done = True

        # Clean the environment for a new generation.
        env.reset()


def run(config_file):
    max_generations = 10

    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)
    
    pop = neat.Population(config)

    stats = neat.StatisticsReporter()

    pop.add_reporter(stats)
    #pop.add_reporter(neat.Checkpointer(generation_interval=25, time_interval_seconds=1200, filename_prefix='NEAT/checkpoints/NEAT-checkpoint-'))
    #pop.add_reporter(neat.TBReporter(True, 1, 1, path="D:\\Projects\\autonomous-drone\\NEAT\\logs\\tensorboard\\"))
    pop.add_reporter(neat.StdOutReporter(True))
    pe = neat.ThreadedEvaluator(4, eval_agent)
    best = pop.run(eval_agent, max_generations)
    pe.stop()
    # Display the winning genome.
    print('\nBest genome:\n{!s}'.format(best))
    print("Finished running!")
    print("Closing environment!")
    env.close()
    
    # Save best genome
    with open(save_nn_destination, 'wb') as f:
        pickle.dump(best, f)

if __name__ == "__main__":
    #atexit.register(exit_handler)
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config_ctrnn')
    if is_training:
        run(config_path)
    else:
        with open(save_nn_destination, "rb") as f:
            genome = pickle.load(f)
            print(genome)
        print(genome.fitness)
        config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)
    
        sim_agent(genome, config)
