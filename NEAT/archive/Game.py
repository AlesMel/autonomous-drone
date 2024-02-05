import numpy as np
import neat
import multiprocessing as mp
import pickle

# MLAGENTS stuff
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

single_agent_env_path = "./Builds/SingleAgent/3DBallBalancing.exe"
multi_agent_env_path = "./Builds/MultipleAgents/3DBallBalancing.exe"
single_config_path = "NEAT/single_config"
multi_config_path = "NEAT/multi_config"
save_nn_destination = 'NEAT/result/best.pkl'

generation = 0
debug_mode = False
is_training = False

class Game:
    def __init__(self, env_name, time_scale = 1.0, width=720, height=480, target_frame_rate=60, quality_level=5, base_port=5006, worker=0, seed=0, no_graphics = False):
        engine_config_channel = EngineConfigurationChannel()
        engine_config_channel.set_configuration_parameters(width=width, height=height, quality_level=quality_level, target_frame_rate=target_frame_rate, time_scale=time_scale)
        
        self.unity_env = UnityEnvironment(base_port=base_port, worker_id=worker, file_name=env_name, side_channels=[engine_config_channel], seed=seed, no_graphics=no_graphics)
        print(f"Enviroment created with id: {base_port+worker}")
        self.reset()
        self.behavior_specs = self.unity_env.behavior_specs
        print(f"Behavior specs: {list(self.behavior_specs)}")
        self.behavior_name = list(self.behavior_specs)[0]
        self.spec = self.behavior_specs[self.behavior_name]
        self.n_agents = len(self.unity_env.get_steps(self.behavior_name)[0])
        print(f"Number of agents {self.n_agents}")
        self.action_size = self.spec.action_spec.continuous_size
    
    def reset(self):
        self.unity_env.reset()
        print("Enviroment has been reset")

    def close(self):
        self.unity_env.close()
        print("Enviroment has been closed")

    def start(self, neural_networks):
        n_networks = len(neural_networks)
        if debug_mode:
            print(f"N_NNS: {n_networks}")

        self.unity_env.reset()
        decision_steps, terminal_steps = self.unity_env.get_steps(self.behavior_name)

        done = [False if i < n_networks else True for i in range(self.n_agents)]
        score = [0 for _ in range (n_networks)]

        while not all(done):
            actions = np.zeros((self.n_agents, 2))  # Assuming each agent expects 2 actions
            nn_input = np.zeros((self.n_agents, 8))

            # Get inputs for NN
            if len(decision_steps) == 0 and len(terminal_steps) == self.n_agents:
                break
            
            for agent in range(self.n_agents):
                if agent in decision_steps:
                    if debug_mode:
                        print(f"{decision_steps[agent].obs[:]}")
                    nn_input[agent] = np.asarray(decision_steps[agent].obs[:])

            # Generate outputs from NN
            for agent in range(self.n_agents):
                if agent in decision_steps:
                    # Generate actions using the neural network
                    if debug_mode:
                        print(f"Observations: {nn_input[agent]}")
                    actions[agent] = neural_networks[agent].activate(nn_input[agent])

            # Perform action
            if len(decision_steps.agent_id) != 0:
                for agent in range(self.n_agents):
                    if agent in decision_steps:
                        continuous_actions = [actions[agent, :]]
                        continuous_actions = np.array(continuous_actions) * 2 - 0.5
                        action_tuple = ActionTuple(discrete=None, continuous=continuous_actions)
                        if debug_mode:
                            print(f"Actions: {continuous_actions}")
                        action_tuple = ActionTuple(discrete=None, continuous=continuous_actions)
                        self.unity_env.set_action_for_agent(behavior_name=self.behavior_name, agent_id=agent, action=action_tuple)

            self.unity_env.step()

            decision_steps, terminal_steps = self.unity_env.get_steps(self.behavior_name)
            if debug_mode:
                print(f"Terminal steps after action: {[ts for ts in terminal_steps]}")
                print(f"Length of Decision steps after action: {len(decision_steps)}")
            if len(decision_steps) > 0:
                score = [score[i]+decision_steps.reward[i] if not done[i] else score[i] for i in range(n_networks)]
            
            for agent in terminal_steps:
                done[agent] = True
            if debug_mode:
                print(f"Done2: {[d for d in done]}")
        print(f"Generation number {generation} done!")
        return score

    # def start(self, genomes, cfg):
    #     global generation
    #     generation += 1
    #     neural_networks = []

    #     # This somehow multiplicates by 2..
    #     print(f"Genomes length: {len(genomes)}")
    #     for _, p in genomes:
    #         p.fitness = 0
    #         neural_network = neat.nn.FeedForwardNetwork.create(p, config=cfg)
    #         neural_networks.append(neural_network)
    
    #     n_networks = len(neural_networks)
    #     print(f"N_NNS: {n_networks}")

    #     self.unity_env.reset()
    #     decision_steps, terminal_steps = self.unity_env.get_steps(self.behavior_name)

    #     done = [False if i < n_networks else True for i in range(self.n_agents)]
    #     score = [0 for _ in range (n_networks)]

    #     while not all(done):
    #         # Get inputs for NN
    #         if len(decision_steps) == 0:
    #             break
            
    #         observations = [decision_steps.obs[0][i] for i in range(self.n_agents)]

    #         actions = np.zeros((self.n_agents, 2))  # Assuming each agent expects 2 actions
            
    #         # Generate outputs from NN
    #         for i in range(self.n_agents):
    #             if not done[i]:
    #                 # Generate actions using the neural network
    #                 neural_output = neural_networks[i].activate(observations[i])
    #                 # Apply the scaling and offset as in your original code
    #                 action = np.array(neural_output) * 2 - 0.5
    #                 # Ensure the action has 2 elements
    #                 actions[i] = action[:2]  # Take only the first 2 actions if there are more
            
    #         # Apply Actions
    #         print(f"Actions: {actions}")
    #         action_tuple = ActionTuple(continuous=actions)
    #         self.unity_env.set_actions(self.behavior_name, action_tuple)

    #         self.unity_env.step()

    #         decision_steps, terminal_steps = self.unity_env.get_steps(self.behavior_name)
    #         print(f"Terminal steps after action: {[ts for ts in terminal_steps]}")
    #         print(f"Length of Decision steps after action: {len(decision_steps)}")
    #         if len(decision_steps) > 0:
    #             score = [score[i]+decision_steps.reward[i] if not done[i] else score[i] for i in range(n_networks)]
    #         for agent in range(self.n_agents):
    #             genomes[agent][1].fitness += score[agent]
    #             print(f"Agent {agent} fitness is: {genomes[agent][1].fitness}")
    #         print(f"TS: {[ts for ts in terminal_steps]} Done: {[d for d in done]}")
    #         for agent in terminal_steps:
    #             done[agent] = True
    #         print(f"Done2: {[d for d in done]}")
    #     print(f"Agent {[agent for agent in range(self.n_agents)]} fitness is: {[genomes[agent][1].fitness for agent in range(self.n_agents)]}")
    #     print(f"Genomes length: {len(genomes)}")
    #     print(f"Generation number {generation} done!")

def single_process_evaluation(env_name, networks, n_agents):
    game = Game(env_name=env_name, time_scale=100,target_frame_rate=-1, quality_level=0, no_graphics=True)
    results = []
    for _ in range(4):
        sub_results = []
        for i in range(0, len(networks), n_agents):
            sub_results += game.start(networks[i:i + n_agents])
        results.append(sub_results)
        if debug_mode:
            print(f"Results: {results}")
    game.close()
    return results  

def multi_process_evaluation(neural_networks, env_name, n_process, n_agents):
    queue = mp.Queue()
    split_networks = np.array_split(neural_networks, n_process)
    # for process in range(n_process):
    #     mp.Process(target=multi_process_eval_job, args={queue, split_networks[process], env_name, process, n_agents})
    jobs = [mp.Process(target=multi_process_eval_job, args=(queue, split_networks[i], env_name, i, n_agents))
            for i in range(n_process)]
    for job in jobs: job.start()
    sub_results = [queue.get() for _ in range(n_process)]
    for job in jobs: job.join()
    sub_results.sort(key=lambda tup: tup[0])
    results = []
    for sub_result in sub_results: results += np.mean(sub_result[1], axis=0).tolist()
    return results

def multi_process_eval_job(queue, neural_networks, env_name, worker, n_agents):
    game = Game(env_name=env_name, worker=worker+1, seed=np.random.randint(0,100), no_graphics=True)
    # let each agent play 4 games
    results = []
    for _ in range(4):
        sub_results = []
        for i in range(0, len(neural_networks), n_agents):
            sub_results += game.start(neural_networks)
        results.append(sub_results)
    queue.put((worker, results))
    game.close()
    return results

def evaluation(genomes, cfg):
    global n_process, generation, env_name

    generation += 1
    neural_networks = []

    # This somehow multiplicates by 2..
    if debug_mode:
        print(f"Genomes length: {len(genomes)}")
    for _, p in genomes:
        p.fitness = 0
        neural_network = neat.nn.RecurrentNetwork.create(p, config=cfg)
        neural_networks.append(neural_network)
    if n_process == 1:
        results = single_process_evaluation(env_name, neural_networks, game.n_agents)
        if debug_mode:
            print(f"Results {results}")
        for i in range(len(neural_networks)):
            result_nn = [result[i] for result in results]
            if debug_mode:
                print(f"Index: {i} results: {result_nn}")
            genomes[i][1].fitness += np.mean(result_nn)
            print(f"Genomes fitness: {genomes[i][1].fitness}")
    else:
        results = multi_process_evaluation(neural_networks, env_name, n_process, game.n_agents)
        for i in range(len(genomes)):
            genomes[i][1].fitness += results[i]
            
def run_agent_sim(genomes, cfg):
    game = Game(env_name=multi_agent_env_path)
    for gen in range(5):
        decision_steps, terminal_steps = game.unity_env.get_steps(game.behavior_name)
        policy = neat.nn.RecurrentNetwork.create(genome, cfg)
        global generation
        generation += 1
        done = False  # For the tracked_agent
        agent_count = len(decision_steps.agent_id)
        agent_id = list(decision_steps)[0]
        while not done:
            actions = np.zeros(shape=(agent_count, 2))
            nn_input = np.concatenate((decision_steps[agent_id].obs[:])) 

            
            print(f"Input for Agent{agent_id}: ", nn_input)
            if len(decision_steps) > 0:  # More steps to take?
                action = policy.activate(nn_input)  # FPass for purple action
                print(f"Action for Agent{agent_id}: ", action)
            if len(decision_steps) > 0:
                # Check if agent requests action:
                continuous_actions = [action[:]]
                game.unity_env.set_action_for_agent(behavior_name=game.behavior_name, agent_id=agent_id, action=ActionTuple(discrete=None, continuous=np.array(continuous_actions)))
            
            game.unity_env.step()
            decision_steps, terminal_steps = game.unity_env.get_steps(behavior_name=game.behavior_name)

            if len(decision_steps) > 0:
                agent_id = list(decision_steps)[0]

            # When whole teams are eliminated, end the generation.
            if len(decision_steps) == 0:
                done = True

        # Clean the environment for a new generation.
        game.reset()


if __name__ == '__main__':
    if is_training:
        env_name = multi_agent_env_path
        game = Game(env_name=multi_agent_env_path)
        game.close()
        # config_path = "NEAT/config_ctrnn"

        n_process = 2

        if n_process > 1:
            config_path = multi_config_path
        else:
            config_path = single_config_path

        config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                        neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
        pop = neat.Population(config)
        pop.add_reporter(neat.Checkpointer(generation_interval=25, time_interval_seconds=1200, filename_prefix='NEAT/checkpoints/NEAT-checkpoint-'))
        # Add reporter for fancy statistical result
        pop.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        pop.add_reporter(stats)
        max_generations = 100
        best_genome = pop.run(evaluation, max_generations)
        # Save best genome.
        with open(save_nn_destination, 'wb') as f:
            pickle.dump(best_genome, f)

    else:
        with open(save_nn_destination, "rb") as f:
            genome = pickle.load(f)
            print(genome)
        print(genome.fitness)
        config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation, single_config_path)

        run_agent_sim(genome, config)

