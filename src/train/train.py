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
    env = gym.wrappers.RecordEpisodeStatistics(env, deque_size=100_000)

    model = PPO("MultiInputPolicy", env, n_steps=1024, verbose=1)
    model = model.load("./rl_model_98000_steps.zip", env)
    checkpoint_callback = CheckpointCallback(save_freq=7000, save_path='.', name_prefix='rl_model')
    model.learn(total_timesteps=int(1e5), callback=checkpoint_callback)