TRAINER_TYPE = "neat"

from mlagents.trainers.trainer.on_policy_trainer import OnPolicyTrainer
from mlagents.trainers.ppo.optimizer_torch import TorchPPOOptimizer, PPOSettings
from mlagents.trainers.settings import TrainerSettings
from mlagents.trainers.trajectory import Trajectory

from neat_optimizer import NEATSettings


class NEATTraienr(OnPolicyTrainer):
    """ The NEAT Trainer implementation """
    def __init__(
            self, 
            behavior_name: str, 
            reward_buff_cap: int,
            trainer_settings: TrainerSettings,
            training: bool,
            load: bool,
            seed: int,
            artifact_path: str,
        ):
        """
            Responsible for collecting experiences and training PPO model.
            :param behavior_name: The name of the behavior associated with trainer config
            :param reward_buff_cap: Max reward history to track in the reward buffer
            :param trainer_settings: The parameters for the trainer.
            :param training: Whether the trainer is set for training.
            :param load: Whether the model should be loaded.
            :param seed: The seed the model will be initialized with
            :param artifact_path: The directory within which to store artifacts from this trainer.
        """
        super().__init__(
            behavior_name,
            reward_buff_cap,
            trainer_settings,
            training,
            load,
            seed,
            artifact_path,
        )
        self.hyperparameters: NEATSettings = cast(
            NEATSettings, self.trainer_settings.hyperparameters
        )
        self.seed = seed
        self.shared_critic = self.hyperparameters.shared_critic
        self.policy: TorchPolicy = None  # type: ignore

    def _process_trajectory(self, trajectory: Trajectory) -> None:
        super()._process_trajectory(trajectory)