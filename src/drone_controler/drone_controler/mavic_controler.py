import rclpy

import time
import math
import numpy as np

from geometry_msgs.msg import Twist, Vector3, Point, PointStamped
from webots_ros2_msgs.msg import FloatStamped

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from drone_interfaces.srv import GetGpsPos
from drone_interfaces.action import MoveRelative
from rclpy.executors import MultiThreadedExecutor

class MavicControler(Node):
    def __init__(self):
        super().__init__('mavic_controler')
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        # subscription_callback_group = MutuallyExclusiveCallbackGroup()

        self.subscription = self.create_subscription(
            PointStamped,
            'Depth_Mavic_2_PRO/gps',
            self.gps_callback,
            10,
            )

        self.subscription = self.create_subscription(
            FloatStamped,
            'Depth_Mavic_2_PRO/compass/bearing',
            self.compass_callback,
            10,
            )
        self.gps_point = Point()
        self.compass_angle = 0

        # Actions
        self.move_relative = ActionServer(self, MoveRelative, 'move_relative', self.move_relative_action)
        self.move_relative_nd = ActionServer(self, MoveRelative, 'move_relative_no_dynamics', self.move_relative_action)
        
        # Services
        
        self.linear_precision = 0.6
        self.angular_precision = 5

        self.get_logger().info(f"-- Mavic controler node created --")

    def compass_callback(self, msg):
        self.compass_angle = msg.data
        # self.get_logger().info(f"compass callback: {self.compass_angle}")
    
    def gps_callback(self, msg):
        self.gps_point = msg.point
        # self.get_logger().info(f"gps callback: {self.gps_point.x} {self.gps_point.y}")

    def publish_drone_vel(self, linear=[0.0,0.0,0.0], angular=0.0):

        cmd_vel = Twist()
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = angular

        cmd_vel.linear.x = linear[0]
        cmd_vel.linear.y = linear[1]
        cmd_vel.linear.z = linear[2]

        self.cmd_vel_publisher.publish(cmd_vel)

    def move_relative_action(self, goal_handle):

        self.get_logger().info(f"-- Move relative action registered. Requested speed: {goal_handle.request.linear_speed} --")
        
        relatve_pos = np.array([goal_handle.request.x, goal_handle.request.y, goal_handle.request.z])
        relative_angle = goal_handle.request.angle

        current_angle = self.compass_angle
        current_pos = np.array([self.gps_point.x, self.gps_point.y, self.gps_point.z])

        target_pos = current_pos + relatve_pos
        target_linear_speed = goal_handle.request.linear_speed
        target_angle = current_angle + relative_angle
        target_rotation_speed = goal_handle.request.rotation_speed

        remaining_distance = np.linalg.norm(relatve_pos)
        remaining_angle = np.abs(relative_angle)

        # Normalize direction to unit len and multiplpy by speed
        if remaining_distance != 0:
            speed_vec = relatve_pos/remaining_distance * target_linear_speed
        else:
            speed_vec = np.array([0.0, 0.0, 0.0])

        
        linear_twist = speed_vec
        if target_angle-current_angle<180 and target_angle-current_angle>0:
            angular_twist = float(-1*target_rotation_speed) 
        else:
            angular_twist = float(target_rotation_speed)

        lin_reached = False
        rot_reached = False

        # Move linear
        self.publish_drone_vel(linear=linear_twist, angular=0.0)
        while remaining_distance > self.linear_precision:
            self.get_logger().info(f"Moving linear... Remaining distance: {remaining_distance}")
            
            current_pos = np.array([self.gps_point.x, self.gps_point.y, self.gps_point.z])            
            remaining_distance = np.linalg.norm(target_pos - current_pos)
            # linear_twist = (target_pos - current_pos)/remaining_distance*target_linear_speed
            # self.publish_drone_vel(linear=linear_twist, angular=0.0)
            
        # Move angular
        self.publish_drone_vel(angular=angular_twist)
        while remaining_angle > self.angular_precision:
            self.get_logger().info(f"Moving angular...  Remaining angle: {remaining_angle}")
            current_angle = self.compass_angle
            angle_diff = np.abs(target_angle-current_angle)
            remaining_angle = min(angle_diff, 360-angle_diff)

            # feedback_msg.distance = float(remaining_distance)
            

        # # Stop
        self.publish_drone_vel(linear=[0.0,0.0,0.0])
        self.get_logger().info(f"Goal reached. Stopping...")
        goal_handle.succeed()
        result = MoveRelative.Result()
        result.result=1
        self.get_logger().info(f"ret")
        return result
def main(args=None):
    rclpy.init(args=args)
    
    mavic_controler = MavicControler()
    executor = MultiThreadedExecutor()
    executor.add_node(mavic_controler)

    try:
        mavic_controler.get_logger().info('Beginning client, shut down with CTRL-C')
        # rclpy.spin(mavic_controler)
        executor.spin()
    except KeyboardInterrupt:
        mavic_controler.get_logger().info('Keyboard interrupt, shutting down.\n')

    # rclpy.spin(mavic_controler)

    mavic_controler.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()