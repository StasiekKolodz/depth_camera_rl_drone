import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
import sys
sys.path.append('/home/stas/Thesis/rl_drone/src/gym_depth_camera_drone')
sys.path.append('/home/stas/Thesis/rl_drone/src')
import gym_depth_camera_drone.envs

if __name__ == '__main__':
    env = gym.make("depth_navigation-v0")
    env = gym.wrappers.RecordEpisodeStatistics(env, deque_size=100_000_0)
    print("Env created")
    model = PPO.load("../train/rl_model_15_1200000_steps.zip")
    print(model.policy)
    print("Model loaded")
    mean_rew, std_rew = evaluate_policy(model, env, n_eval_episodes=1000, deterministic=True)
    print(f"Mean reward: {mean_rew}, std reward: {std_rew}")
    print(f"Success counter {env.success_counter}")
    print(f"Ep counter {env.ep_counter}")
    