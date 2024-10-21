import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from drone_interfaces.action import MoveRelative



class MoveDroneClient(Node):

    def __init__(self):
        super().__init__('move_drone_client')
        self._action_client = ActionClient(self, MoveRelative, 'move_relative')
        # self.timer = self.create_timer(40, self.send_goal_param)
    def send_goal_param(self):
        self.get_logger().info(f"send goal param")
        self.send_goal([0, 1, 0], 0.1)

    def send_goal(self, rel_pos, speed, rel_angle, angular_speed):
        self.get_logger().info(f"-- Sending move_relative action goal. rel_pos={rel_pos} speed={speed} --")
        goal_msg = MoveRelative.Goal()
        goal_msg.x = float(rel_pos[0])
        goal_msg.y = float(rel_pos[1])
        goal_msg.z = float(rel_pos[2])
        goal_msg.linear_speed = float(speed)
        goal_msg.angle = float(rel_angle)
        goal_msg.rotation_speed = float(angular_speed)
        self.get_logger().info(f"wait server")
        self._action_client.wait_for_server()
        self.get_logger().info(f"wait server2")
        self.send_goal_future = self._action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.move_rel_response_callback)

    def move_rel_response_callback(self, future):
        self.get_logger().info("Move rel response callback")
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.move_rel_result_callback)

    def move_rel_result_callback(self, future):
        self.get_logger().info("Move rel  action finished")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = MoveDroneClient()

    action_client.send_goal([0.5, 0, 0], 0.15, -90, 0.3)
    action_client.get_logger().info(f"spin until future")
    rclpy.spin(action_client)
    # rclpy.spin_until_future_complete(action_client, future)
    
    action_client.destroy_node()

    # rclpy.shutdown()


if __name__ == '__main__':
    main()