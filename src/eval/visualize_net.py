from torchviz import make_dot
from stable_baselines3 import PPO

# Load the model
model = PPO.load("../train/rl_model_15_1200000_steps.zip")

import hiddenlayer as hl

transforms = [ hl.transforms.Prune('Constant') ] # Removes Constant nodes from graph.

graph = hl.build_graph(model, batch.text, transforms=transforms)
graph.theme = hl.graph.THEMES['blue'].copy()
graph.save('rnn_hiddenlayer', format='png')
# # Visualize the network
# dot = make_dot(y, params=dict(model.named_parameters()))
# dot.format = 'png'
# dot.render('network_visualization')