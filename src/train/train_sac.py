import os
# from pathlib import Path

import gymnasium as gym
import numpy as np

from stable_baselines3 import SAC
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
    env = gym.make("depth_navigation-v0", discrete_action_space=False)
    env = gym.wrappers.RecordEpisodeStatistics(env)

    tensorboard_log = "./sac_tensorboard_log"
    # eval_callback = EvalCallback(eval_env, best_model_save_path="./saved_models/",
    #                          eval_freq=1024,
    #                          deterministic=True, render=False, verbose=1, n_eval_episodes=3)
    def linear_schedule(initial_value: float) -> Callable[[float], float]:
        def func(progress_remaining: float) -> float:
            return progress_remaining * initial_value
        return func

    model = SAC("MultiInputPolicy",env, 
                learning_rate=linear_schedule(0.0001), 
                verbose=1, 
                tensorboard_log=tensorboard_log)
    print(model.policy)
    # model = SAC.load("./rl_model_23_1000000_steps.zip", env, tensorboard_log=tensorboard_log, custom_objects=load_kwargs)
    checkpoint_callback = CheckpointCallback(save_freq=50_000, save_path='./saved_models/SAC', name_prefix='sac_model_3')
    model.learn(total_timesteps=int(0.11e6), callback=checkpoint_callback, progress_bar=True)