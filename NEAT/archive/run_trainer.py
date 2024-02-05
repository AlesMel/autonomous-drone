from multiprocessing import Value, Lock
from enviroment import *
from agent import Agent   
from trainer import NeatTrainer
import neat
import time
import random

id = 0

worker_id_counter = Value('i', 0)
worker_id_lock = Lock()

def create_unique_worker_id():
    # Use a combination of current time and a random component
    return int(time.time() * 0) + random.randint(0, 999)

def eval_genome(genome, config):
    worker_id = create_unique_worker_id()
    env = create_enviroment(env_path="./Builds/single-agent-env/autonomous-drone.exe", worker_id=worker_id, is_training=True)
    agent = Agent(env)
    genome.fitness = agent.process_epoch(genome, config)

if __name__ == "__main__":
    pop_size = 1
    trainer = NeatTrainer(config_path="NEAT/config_ctrnn")
    trainer.run_training(is_parallel=True, fitness=eval_genome, max_generations=10)
