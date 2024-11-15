import os
# from pathlib import Path

import gymnasium as gym
import numpy as np

from stable_baselines3 import PPO
# from stable_baselines3.common.monitor import Monitor
# from stable_baselines3.common.vec_env import DummyVecEnv
# from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.callbacks import CheckpointCallback, ProgressBarCallback, EvalCallback

# from gym_depth_planning.utils import sys_utils
import sys
from typing import Callable

# adding Folder_2 to the system path
# sys.path.insert(0, '/home/stas/Thesis/rl_drone/src/gym_depth_camera_drone')
sys.path.append('~/Thesis/rl_drone/src/gym_depth_camera_drone')
sys.path.append('/home/stas/Thesis/rl_drone/src/gym_depth_camera_drone')
sys.path.append('/home/stas/Thesis/rl_drone/src')
import gym_depth_camera_drone.envs  
from stable_baselines3.common.vec_env import vec_transpose

if __name__ == '__main__':
    env = gym.make("depth_navigation-v0")

    # env = Monitor(env, filename=logdir, allow_early_resets=True)
    # env = DummyVecEnv([lambda: env])
    env = gym.wrappers.RecordEpisodeStatistics(env)

    tensorboard_log = "./ppo_tensorboard_log"

    policy_kwargs = {
    "net_arch": {
        "pi": [1024,512,256],
        "vf": [1024,512,256],
        }
    }

    def linear_schedule(initial_value: float) -> Callable[[float], float]:
        def func(progress_remaining: float) -> float:
            return progress_remaining * initial_value
        return func

    # eval_callback = EvalCallback(eval_env, best_model_save_path="./saved_models/",
    #                          eval_freq=1024,
    #                          deterministic=True, render=False, verbose=1, n_eval_episodes=3)
    
    # model = PPO("MultiInputPolicy", env, gamma=0.95,n_steps=2048*2, batch_size=256, verbose=1,learning_rate=linear_schedule(0.0004), policy_kwargs=policy_kwargs, tensorboard_log=tensorboard_log)
    model = PPO.load("./saved_models/rl_model_23_1000000_steps.zip", env, tensorboard_log=tensorboard_log)
    checkpoint_callback = CheckpointCallback(save_freq=200_000, save_path='./saved_models', name_prefix='rl_model_23_3')
    model.learn(total_timesteps=int(1.6e6), callback=checkpoint_callback, progress_bar=True)