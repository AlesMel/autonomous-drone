import numpy as np
import neat
from mlagents_envs.base_env import ActionTuple

class Agent:
    def __init__(self, env):
        self.env = env
        self.behavior_name = list(self.env.behavior_specs)[0]
        self.spec = self.env.behavior_specs[self.behavior_name]
        self.num_observations = self.spec.observation_specs
        self.visual_observations = any(len(spec.shape) == 3 for spec in self.num_observations)
        if self.spec.action_spec.continuous_size > 0:
            self.continous_actions = self.spec.action_spec.continuous_size
        if self.spec.action_spec.is_discrete():
            self.discrete_actions = self.spec.action.spec.discrete_size

    def process_epoch(self, genome, config):
        policy = neat.nn.RecurrentNetwork.create(genome, config)
        decision_steps, terminal_steps = self.env.get_steps(self.behavior_name)
        agent_id = decision_steps
        done = False
        fitness = 0
        while not done:
            if decision_steps:
                inputs = np.asarray(decision_steps[0].obs[:])
                print(inputs[0])
                actions = policy.activate(inputs[0])
                print(np.array(actions[:]))
                continuous_actions = np.array(actions[:]).reshape(1, len(actions))
                action_tuple = ActionTuple(discrete=None, continuous=continuous_actions)
                self.env.set_action_for_agent(behavior_name=self.behavior_name, agent_id=agent_id, action=action_tuple)
                self.env.step()
                decision_steps, terminal_steps = self.env.get_steps(self.behavior_name)
                if terminal_steps:
                    fitness+=terminal_steps.reward
                    done = True
                elif decision_steps:
                    fitness+=decision_steps.reward
        self.env.reset()
        return fitness
