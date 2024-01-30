import neat

class NEATTrainer:
    def __init__(self, config_path):
        config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
        self.pop = neat.Population(config)
        self.__set_reporters()

    def __set_reporters(self):
        self.pop.add_reporter(neat.Checkpointer(generation_interval=25, time_interval_seconds=1200, filename_prefix='NEAT/checkpoints/NEAT-checkpoint-'))
        self.pop.add_reporter(neat.StdOutReporter(True))
        self.pop.add_reporter(neat.TBReporter(False, 0, 0, path="D:\\Projects\\autonomous-drone\\NEAT\\logs\\tensorboard\\"))
        stats = neat.StatisticsReporter()
        self.pop.add_reporter(stats)

    def run_training(self, is_parallel, fitness, max_generations):
        if is_parallel:
            pe = neat.ParallelEvaluator(6, fitness)
            best = self.pop.run(pe.evaluate, max_generations)
        else:
            best = self.pop.run(fitness, max_generations)
        return best