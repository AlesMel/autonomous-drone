import pickle
import sys
import numpy as np
import time
import atexit
import neat
import visualize
import math

# MLAGENTS stuff
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

save_nn_destination = 'NEAT/result/best.pkl'
result_destination= 'NEAT/result/in_progres/best_genome5.pkl'

# [PARAMETERS]  
max_generations = 100 # Max number of generations
is_training = True   
no_graphics = False
is_multi = True
is_debug = False
load_from_checkpoint = False

generation = 0

# Best
best_current_gen = None

single_agent_env_path = "./Builds/single-agent-env/autonomous-drone.exe"
multi_agent_env_path = "./Builds/autonomous-drone.exe"

engine_config_channel = EngineConfigurationChannel()

if is_training:
    engine_config_channel.set_configuration_parameters(width=160, height=90, quality_level=0, time_scale=100)
else:
    engine_config_channel.set_configuration_parameters(width=2048 , height=1080)

if is_multi:
    env = UnityEnvironment(file_name=multi_agent_env_path, worker_id=6, seed=0, no_graphics = no_graphics, side_channels=[engine_config_channel])
else:   
    env = UnityEnvironment(file_name=single_agent_env_path, worker_id=5, seed=0, no_graphics = no_graphics)

# Reset the enviroment to get it ready  
print("ENV Has been reset")
env.reset()

behavior_specs = env.behavior_specs
behavior_name = list(behavior_specs)[0]
print(f"Name of the behavior : {behavior_name}")
spec = env.behavior_specs[behavior_name]

# Examine the number of observations per Agent
print("Number of observations : ", len(spec.observation_specs))

# Is there a visual observation ?
# Visual observation have 3 dimensions: Height, Width and number of channels
vis_obs = any(len(spec.shape) == 3 for spec in spec.observation_specs)
print("Is there a visual observation ?", vis_obs)

# Is the Action continuous or multi-discrete ?
if spec.action_spec.continuous_size > 0:
  print(f"There are {spec.action_spec.continuous_size} continuous actions")
if spec.action_spec.is_discrete():
  print(f"There are {spec.action_spec.discrete_size} discrete actions")

# For discrete actions only : How many different options does each action has ?
if spec.action_spec.discrete_size > 0:
  for action, branch_size in enumerate(spec.action_spec.discrete_branches):
    print(f"Action number {action} has {branch_size} different options")

def exit_handler():
    # visualize.plot_stats(stats, view=True, filename="NEAT/result/in_progress/recurrent-fitness"+str(generation)+".svg", label="CTRNN")
    # visualize.plot_species(stats, view=True, filename="NEAT/result/in_progress/recurrent-speciation"+str(generation)+".svg", label="CTRNN")
    with open(save_nn_destination, 'wb') as w:
        pickle.dump(best_current_gen, w)
    print("EXITING")
    env.close

atexit.register(exit_handler)

def run_agent(genomes, cfg):
    """
    Population size is configured as 12 to suit the training environment!
    :param genomes: All the genomes in the current generation.
    :param cfg: Configuration file
    :return: Best genome from generation.
    """
    decision_steps, terminal_steps = env.get_steps(behavior_name)
    agent_count = len(decision_steps.agent_id) # 2 agents in multiagent-env

    policies = [] # save neural networks

    #  150 pop size
    for _, g in genomes:
        g.fitness = 0
        policy = neat.nn.RecurrentNetwork.create(g, cfg)
        policies.append(policy)

    done = False # keep track of termination
    while not done:
        actions = np.ones(shape=(agent_count, 4))
        nn_input = np.zeros(shape=(agent_count, 17))
        for agent in decision_steps:
            observation = decision_steps[agent].obs
            nn_input[agent] = np.concatenate(observation)
        for policy in policies:
            for agent in decision_steps:
                actions[agent] = policy.activate(nn_input[agent])



if __name__ == "__main__":
    config_path = "NEAT/config_ctrnn"
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
    if is_training:
        pop = neat.Population(config)
        # For saving checkpoints during training    Every 25th generation or 20 minutes
        pop.add_reporter(neat.Checkpointer(generation_interval=25, time_interval_seconds=1200, filename_prefix='NEAT/checkpoints/NEAT-checkpoint-'))
        # Add reporter for fancy statistical result
        pop.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        pop.add_reporter(stats)
        evaluation = run_agent
        best_genome = pop.run(evaluation, max_generations)