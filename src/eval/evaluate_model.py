import gymnasium as gym
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.evaluation import evaluate_policy
import sys
sys.path.append('/home/stas/Thesis/rl_drone/src/gym_depth_camera_drone')
sys.path.append('/home/stas/Thesis/rl_drone/src')
import gym_depth_camera_drone.envs


if __name__ == '__main__':
    env = gym.make("depth_navigation-v0", discrete_action_space=False, no_dynamics=True)
    env = gym.wrappers.RecordEpisodeStatistics(env)
    print("Env created")
    model = PPO.load("../train/saved_models/PPO/ppo_continous_1_20000000_steps.zip")
    # model = SAC.load("../train/saved_models/SAC/dionizos_sac_model_2_1200000_steps.zip")
    print(model.policy)
    print("Model loaded")

    # obs, info = env.reset()
    # while True:
    #     action, states = model.predict(obs)
    #     print(f"Action: {action}")
    #     print(f"States: {states}")
    #     # v = model.policy.predict_values(obs_as_tensor(obs, model.device))
    #     actions, values, log_probs = model.policy.forward(obs_as_tensor(obs, model.device), deterministic=True)
    #     print(f"Value: {values}")
    #     if dones:
    #         obs, info = env.reset()
        
    mean_rew, std_rew = evaluate_policy(model, env, n_eval_episodes=1000, deterministic=True)
    print(f"Mean reward: {mean_rew}, std reward: {std_rew}")
    print(f"Success counter {env.unwrapped.success_counter}")
    print(f"Ep counter {env.unwrapped.ep_counter}")
    