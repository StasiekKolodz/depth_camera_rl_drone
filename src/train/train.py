import os
# from pathlib import Path

import gymnasium as gym
import numpy as np

from stable_baselines3 import PPO
# from stable_baselines3.common.monitor import Monitor
# from stable_baselines3.common.vec_env import DummyVecEnv
# from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.callbacks import CheckpointCallback

# from gym_depth_planning.utils import sys_utils
import sys
from typing import Callable

# adding Folder_2 to the system path
# sys.path.insert(0, '/home/stas/Thesis/rl_drone/src/gym_depth_camera_drone')
sys.path.append('~/Thesis/rl_drone/src/gym_depth_camera_drone')
sys.path.append('/home/stas/Thesis/rl_drone/src/gym_depth_camera_drone')
sys.path.append('/home/stas/Thesis/rl_drone/src')
import gym_depth_camera_drone.envs  

if __name__ == '__main__':
    env = gym.make("depth_navigation-v0")

    # env = Monitor(env, filename=logdir, allow_early_resets=True)
    # env = DummyVecEnv([lambda: env])
    env = gym.wrappers.RecordEpisodeStatistics(env, )

    tensorboard_log = "./ppo_tensorboard_log"

    # hidden_size = 1024
    policy_kwargs = {
    "net_arch": {
        "pi": [1024,512,256],
        "vf": [1024,512,256],
        }
    }

    # def linear_schedule(initial_value: float) -> Callable[[float], float]:
    #     """
    #     Linear learning rate schedule.

    #     :param initial_value: Initial learning rate.
    #     :return: schedule that computes
    #     current learning rate depending on remaining progress
    #     """
    #     def func(progress_remaining: float) -> float:
    #         """
    #         Progress will decrease from 1 (beginning) to 0.

    #         :param progress_remaining:
    #         :return: current learning rate
    #         """
    #         l_rate = progress_remaining * initial_value
    #         if l_rate < 0.0003:
    #             return 0.0003
    #         else:
    #             return l_rate

    #     return func
    # model = PPO("MultiInputPolicy", env, gamma=0.9, verbose=1, policy_kwargs=policy_kwargs, tensorboard_log=tensorboard_log)
    model = PPO.load("./rl_model_15_1200000_steps.zip", env, tensorboard_log=tensorboard_log)
    checkpoint_callback = CheckpointCallback(save_freq=200_000, save_path='.', name_prefix='rl_model_15_2')
    model.learn(total_timesteps=int(10e6), callback=checkpoint_callback)