import rclpy


class GhostRobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot

    def step(self):
        pass
