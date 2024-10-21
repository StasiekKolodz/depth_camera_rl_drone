import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci

import math
import numpy as np

from geometry_msgs.msg import Twist, Vector3, Point, PointStamped

from drone_interfaces.srv import GetGpsPos
from drone_interfaces.action import MoveRelative
from rclpy.executors import MultiThreadedExecutor

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Actions
        self.move_relative = ActionServer(self, MoveRelative, 'move_relative', self.move_relative_action)

        # Services
        self.gps_position_cli = self.create_client(GetGpsPos, 'gps_position')

        self.subscription = self.create_subscription(
            PointStamped,
            'Depth_Mavic_2_PRO/gps',
            self.gps_callback,
            10)
        self.gps_point = Point()

        self.get_logger().info('Action server creasted...')

    def gps_callback(self, msg):
        self.gps_point = msg.point

    def request_gps_position(self):
        # self.get_logger().info('Sending GPS position request')
        self.get_logger().info(f"request gps...")
        request = GetGpsPos.Request()
        
        future = self.gps_position_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"request gps complete.")
        return future.result()

    def publish_drone_vel(self, linear, angular=[0.0,0.0,0.0]):
        self.get_logger().info(f"pub...")

        cmd_vel = Twist()
        cmd_vel.angular.x = angular[0]
        cmd_vel.angular.y = angular[1]
        cmd_vel.angular.z = angular[2]

        cmd_vel.linear.x = linear[0]
        cmd_vel.linear.y = linear[1]
        cmd_vel.linear.z = linear[2]

        self.cmd_vel_publisher.publish(cmd_vel)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            # gps_response = self.request_gps_position()
            rclpy.spin_once(self)
            self.publish_drone_vel(linear=[1.0,1.0,1.0])
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info('returning...')
        return result

    def move_relative_action(self, goal_handle):

        self.get_logger().info(f"-- Move relative action registered. Requested speed: {goal_handle.request.speed} --")
        
        relatve_pos = np.array([goal_handle.request.x, goal_handle.request.y, goal_handle.request.z])

        gps_response = self.request_gps_position()
        current_pos = np.array([gps_response.x, gps_response.y, gps_response.z])

        target_pos = current_pos + relatve_pos
        target_speed = goal_handle.request.speed

        remaining_distance = np.linalg.norm(relatve_pos)
        # Normalize direction to unit len and multiplpy by speed
        speed_vec = relatve_pos/remaining_distance * target_speed

        # feedback_msg = MoveRelative.Feedback()
        # Move
        # feedback_msg.distance = float(remaining_distance)
        self.publish_drone_vel(linear=-1*speed_vec)
        while remaining_distance > 0.1:
            self.get_logger().info(f"Moving... Remaining distance: {remaining_distance}")
            gps_response = self.request_gps_position()
            current_pos = np.array([gps_response.x, gps_response.y, gps_response.z])
            remaining_distance = np.linalg.norm(target_pos - current_pos)
            # feedback_msg.distance = float(remaining_distance)
            
            #Update direction
            # speed_vec = (target_pos - current_pos)/remaining_distance * target_speed
            # self.publish_drone_vel(linear=-1*speed_vec)
        # # Stop
        self.publish_drone_vel(linear=[0.0,0.0,0.0])
        self.get_logger().info(f"Goal reached. Stopping...")
        result = MoveRelative.Result()
        result.result=1
        goal_handle.succeed()
        self.get_logger().info(f"ret")
       
        return result

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()