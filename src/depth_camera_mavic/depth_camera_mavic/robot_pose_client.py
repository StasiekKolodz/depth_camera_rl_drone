import sys

from drone_interfaces.srv import SetRobotPose
import rclpy
from rclpy.node import Node


class RobotPoseClient(Node):

    def __init__(self):
        super().__init__('robot_pose_client')
        self.cli = self.create_client(SetRobotPose, 'set_robot_pose')
        
        self.req = SetRobotPose.Request()

    def send_request(self):
        self.req.robot_def = "Depth_Mavic_2_PRO"
        self.req.translation.x = 1.0
        self.req.translation.y = 0.0
        self.req.translation.z = 1.0
        self.req.rotation.x = 0.0
        self.req.rotation.y = 0.0
        self.req.rotation.z = 1.0
        self.req.rotation.angle = 0.0
        self.req.reset_physics = False
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    pose_client = RobotPoseClient()
    response = pose_client.send_request()

    pose_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()