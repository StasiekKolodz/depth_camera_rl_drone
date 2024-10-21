import cv2
import numpy as np

class EnvVisualizer:
    def __init__(self, env_boundaries):
        self.env_boundaries = env_boundaries
        self.image = np.zeros((env_boundaries[1]*2, env_boundaries[0]*2, 3), dtype=np.uint8)

    def update(self, drone_position, goal_position):
        drone_position = (int(drone_position[0]), int(drone_position[1]))
        goal_position = (int(goal_position[0]), int(goal_position[1]))
        self.image.fill(0)
        cv2.circle(self.image, drone_position, 2, (255, 0, 0), -1)  # Drone position in blue
        cv2.circle(self.image, goal_position, 2, (0, 255, 0), -1)  # Goal position in green
        cv2.line(self.image, drone_position, goal_position, (0, 0, 255), 1)  # Vector in red
        # cv2.rectangle(image, start_point, end_point, color, thickness)

    def show(self):
        cv2.imshow('Environment Visualization', self.image)
        cv2.waitKey(1)

    def save(self, filename):
        cv2.imwrite(filename, self.image)

# Example usage:
# env_viz = EnvVisualizer((640, 480))
# env_viz.update((100, 100), (200, 200))
# env_viz.show()
# env_viz.save('env_visualization.png')