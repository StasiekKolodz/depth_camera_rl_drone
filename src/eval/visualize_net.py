from torchviz import make_dot
from stable_baselines3 import PPO
import torch
import os
import sys
import gymnasium as gym

src_dir_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(src_dir_path)
print(src_dir_path)
import gym_depth_camera_drone.envs  

env = gym.make("depth_navigation-v0")
observation, info = env.reset()
model = PPO.load("../train/saved_models/PPO/rl_model_15_1200000_steps.zip")


x = model.policy.obs_to_tensor(observation)[0]
output = model.policy.forward(x)

make_dot = make_dot(output, params=dict(model.policy.named_parameters()))
make_dot.render("policy_network", format="png")  # Saves to a file called 'policy_network.png'

# self.policy.obs_to_tensor(infos[idx]["terminal_observation"])[0]

import hiddenlayer as hl

transforms = [ hl.transforms.Prune('Constant') ] # Removes Constant nodes from graph.
graph = hl.build_graph(model.policy, x, transforms=transforms)
graph.theme = hl.graph.THEMES['blue'].copy()
# graph.save('rnn_hiddenlayer', format='pdf')
# # # Visualize the network
# # dot = make_dot(y, params=dict(model.named_parameters()))
# # dot.format = 'png'
# # dot.render('network_visualization')