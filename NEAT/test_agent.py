import pickle
import numpy as np
import neat
import os
import keyboard

#from game_config import game_config
# MLAGENTS stuff
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

is_training = False
env_path = "./Builds/test-side-channel-env/autonomous-drone.exe"
save_nn_destination = 'NEAT/result/best.pkl'

engine_config_channel = EngineConfigurationChannel()
engine_config_channel.set_configuration_parameters(time_scale=1)

# file_name=env_path, seed=0, no_graphics=False, 
env = UnityEnvironment(side_channels=[engine_config_channel])
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

def eval_agent(genome, cfg):
    while True:
        env.reset()
        policy = neat.nn.FeedForwardNetwork.create(genome, cfg)
        decision_steps, terminal_steps = env.get_steps(behavior_name)
        agent_count = len(decision_steps.agent_id)
        policies = [policy] * agent_count

        unity_to_neat_map, neat_to_unity_map = map_agent_ids(decision_steps)

        done = False  # Vectorized initialization
        removed_agents = []

        episode_rewards = [0] * agent_count
        print(f"Agent count: {agent_count}")
        while not done:
            if keyboard.is_pressed('q'):  # Check if 'q' is pressed
                env.close()
                return
            for agent in decision_steps:
                if agent not in removed_agents:
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
                    

            if len(removed_agents) >= agent_count:
                print(".") 
                done = True
        print(episode_rewards[0])
        env.reset()


if __name__ == "__main__":
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'test_config')
    with open(save_nn_destination, "rb") as f:
        genome = pickle.load(f)
        print(genome)
    print(genome.fitness)
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                        neat.DefaultSpeciesSet, neat.DefaultStagnation,
                        config_path)

    eval_agent(genome, config)
