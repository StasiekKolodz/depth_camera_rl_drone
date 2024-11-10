import gymnasium as gym

from rclpy.utilities import ok, get_default_context
import rclpy
from drone_interfaces.srv import SetRobotPose, SetRobotPoseRelative
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from webots_ros2_msgs.msg import FloatStamped
import math
from rclpy.action import ActionClient
from drone_interfaces.action import MoveRelative
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3, Point, PointStamped

from env_randomizer.env_randomizer import EnvRandomizer
from utils.env_vizualizer import EnvVisualizer
class DepthCameraDroneNavigation_v0(gym.Env):
    
    def __init__(self, discrete_action_space=True, no_dynamics=True):
        self.webots_drone_def = "Depth_Mavic_2_PRO"

        self.depth_image_shape = (64, 64, 1)
        self.camera_fov = 1.5184
        self.camera_range = 10
        self.success_counter = 0
        self.ep_counter = 0
        # Gymnasium
        self.discrete_action_space = discrete_action_space
        self.observation_space = gym.spaces.Dict(
            {'depth_image': gym.spaces.Box(low=0, high=255, shape=self.depth_image_shape, dtype=np.uint8),
             'target_vec': gym.spaces.Box(low=-np.inf, high=np.inf, shape=(2,), dtype=np.float64)})

        if self.discrete_action_space:
            self.action_space = gym.spaces.Discrete(7)
        else:
            self.action_space = gym.spaces.Box(low=-1, high=+1, shape=(2,), dtype=np.float64)

        # Init enviroment randomizer (randomize obstacles and goal)
        self.boundary_shape = [10, 12]
        self.env_randomizer = EnvRandomizer(boundary_shape=self.boundary_shape)
        self.goal_point = self.env_randomizer.randomize_goal_point()
        # self.goal_point = np.array([5, 0])
        # ROS interface
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('drone_gym_env')
        self.__node.create_subscription(Bool, 'Depth_Mavic_2_PRO/touch_sensor_warning', self.ts_warning_callback, 1)
        self.__node.create_subscription(Bool, 'Depth_Mavic_2_PRO/touch_sensor_colision', self.ts_colision_callback, 1)
        self.__node.create_subscription(Image, 'Depth_Mavic_2_PRO/range_finder/image', self.depth_image_callback, 1)
        self.__node.create_subscription(PointStamped, 'Depth_Mavic_2_PRO/gps', self.gps_callback, 1)
        self.__node.create_subscription(FloatStamped, 'Depth_Mavic_2_PRO/compass/bearing', self.compass_callback, 1)
        # self.__node.create_subscription(Bool, 'Depth_Mavic_2_PRO/range_finder/point_cloud', self.point_cloud_callback, 10)
        self.robot_pose_cli = self.__node.create_client(SetRobotPose, 'set_robot_pose')
        self.robot_pose_relative_cli = self.__node.create_client(SetRobotPoseRelative, 'set_robot_pose_relative')
        self.move_drone_cli = ActionClient(self.__node, MoveRelative, 'move_relative')
        
        self.cv_bridge = CvBridge()
        self.current_frame = np.zeros((self.depth_image_shape), dtype=np.float32)
        
        self.current_target_vec = np.squeeze(np.zeros((2,1)))

        self.gps_position = np.zeros((1,3))
        self.compass_angle = 0

        self.warning_flag = False
        self.colision_flag = False

        self.action_dictionary = {0: (1, 1),
                                  1: (1, 0),
                                  2: (0, 1),
                                  3: (0, 0),
                                  4: (0, -1),
                                  5: (-1, 0),
                                  6: (-1, -1)}

        # Drone parameters and values
        self.step_length = 1  # meter

        self.drone_busy = False
        self.no_dynamics = no_dynamics

        self.drone_altitude = 2 #meters
        self.drone_init_translation = [0, 0, self.drone_altitude]
        self.drone_init_rotation = [0, 0, 1, 0]

        self.steps_counter = 0

        self.env_visualizer = EnvVisualizer(self.boundary_shape)
        print(f"goal: {self.goal_point}")
    
    # def wait_ros_services(self):
    #     while not self.robot_pose_cli.wait_for_service(timeout_sec=1.0):
    #         self.__node.get_logger().info('set_robot_pose service not available, waiting again...')
    #     while not self.robot_pose_relative_cli.wait_for_service(timeout_sec=1.0):
    #         self.__node.get_logger().info('set_robot_pose_relative service not available, waiting again...')
    #     while not self.move_drone_cli.wait_for_server(timeout_sec=1.0):
    #         self.__node.get_logger().info('move_relative service not available, waiting again...')

    def rotate_vector(self, vector, angle):
        x = vector[0] * math.cos(angle) - vector[1] * math.sin(angle)
        y = vector[0] * math.sin(angle) + vector[1] * math.cos(angle)
        return np.array([x, y])

    def move_drone_no_dynamics(self, translation, yaw_rotation, steps_number=1, reset_physics=False):
        step_translation = np.array(translation)/steps_number
        for step in range(steps_number):
            self.set_drone_pose_relative(step_translation, 0, reset_physics)
            
        self.set_drone_pose_relative([0,0,0], yaw_rotation, reset_physics)

    def set_drone_pose_relative(self, translation, yaw_rotation=0, reset_physics=False):

        # self.__node.get_logger().info(f"sending translation: {translation} rotation: {rotation_axis_angle}-")
        request = SetRobotPoseRelative.Request()
        request.robot_def = self.webots_drone_def
        request.x = float(translation[0])
        request.y = float(translation[1])
        request.altitude = float(self.drone_altitude)
        request.yaw_rotation = float(yaw_rotation)
        request.reset_physics = reset_physics
        future = self.robot_pose_relative_cli.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future)
        return future.result()

    def set_drone_pose(self, translation, rotation_axis_angle, reset_physics=False):
        request = SetRobotPose.Request()
        request.robot_def = self.webots_drone_def
        request.translation.x = float(translation[0])
        request.translation.y = float(translation[1])
        request.translation.z = float(translation[2])
        request.rotation.x = float(rotation_axis_angle[0])
        request.rotation.y = float(rotation_axis_angle[1])
        request.rotation.z = float(rotation_axis_angle[2])
        request.rotation.angle = float(rotation_axis_angle[3])
        request.reset_physics = reset_physics
        future = self.robot_pose_cli.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future)
        return future.result()

    def move_relative(self, rel_pos, speed, rel_angle, angular_speed):
        self.drone_busy = True
        self.__node.get_logger().info(f"-- Sending move_relative action goal. rel_pos={rel_pos} speed={speed} --")
        goal_msg = MoveRelative.Goal()
        goal_msg.x = float(rel_pos[0])
        goal_msg.y = float(rel_pos[1])
        goal_msg.z = float(rel_pos[2])
        goal_msg.linear_speed = float(speed)
        goal_msg.angle = float(rel_angle)
        goal_msg.rotation_speed = float(angular_speed)
        self.__node.get_logger().info(f"wait server")
        self.move_drone_cli.wait_for_server()
        self.send_goal_future = self.move_drone_cli.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.move_rel_response_callback)

    def move_rel_response_callback(self, future):
        self.__node.get_logger().info("Move rel response callback")
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.move_rel_result_callback)

    def move_rel_result_callback(self, future):
        self.__node.get_logger().info("Move rel  action finished")
        self.drone_busy = False

    def depth_image_callback(self, msg):
        # self.__node.get_logger().info(f"Depth image callback")
        self.current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
        self.current_frame = self.clip_depth_image(self.current_frame, self.camera_range)
        # self.__node.get_logger().info(f"{np.min(self.current_frame)}, {np.max(self.current_frame)}")
        # cv2.imshow("depth camera", self.current_frame/255)
        # cv2.waitKey(1) 

    def compass_callback(self, msg):
        self.compass_angle = msg.data

    def gps_callback(self, msg):
        gps_point = msg.point
        self.gps_position = np.array([gps_point.x, gps_point.y, gps_point.z])

    def clip_depth_image(self, image, camera_range):
        # Clip image and rescale values to [0,255] range
        image = np.clip(image, 0, camera_range) / camera_range * 255
        return image.reshape(self.depth_image_shape[0], self.depth_image_shape[1], 1).astype(np.uint8)

    def ts_warning_callback(self, msg):
        if msg.data:
            self.warning_flag = True
    
    def ts_colision_callback(self, msg):
        if msg.data:
            self.colision_flag = True
    
    def wait_ready(self):
        while self.drone_busy:
            rclpy.spin_once(self.__node, timeout_sec=0.1)

    def get_obs(self):
        rclpy.spin_once(self.__node, timeout_sec=0.1)
        target_vec_global = self.goal_point - self.gps_position[0:2]
        # Rotate target vector to drone local frame
        target_vec = self.rotate_vector(target_vec_global, np.radians(self.compass_angle))
        observation = {'depth_image': np.copy(self.current_frame), 'target_vec': np.copy(target_vec)}
        return observation


    def is_reached_goal(self):
        return  self.distance_to_goal() < self.step_length

    def distance_to_goal(self):
        return  np.linalg.norm(np.array(self.goal_point) - np.array(self.gps_position[0:2]))

    def get_reward(self):
        pass

    def get_drone_step(self, action):
        if self.discrete_action_space:
            action = self.action_dictionary[action]
        forward_step = np.cos(action[0] * 22.5 / 180 * np.pi)
        side_step = np.sin(action[0] * 22.5 / 180 * np.pi)
        yaw_step = action[1] * 22.5 / 180 * np.pi
        return [self.step_length * forward_step, self.step_length * side_step, 0], yaw_step
    
    def step(self, action):
        # rclpy.spin(self.__node)
        # self.__node.get_logger().info("Gym enviroment step")
        
        
        trans_step, yaw_step = self.get_drone_step(action)
        # self.__node.get_logger().info(f"Step: {trans_step}, {yaw_step}")
        if self.no_dynamics:
            self.move_drone_no_dynamics(trans_step, yaw_step, steps_number=3, reset_physics=False)
        else:
            print("Dynamics not implemented")
            # TODO: Implement movement with dynamicss
            # self.move_relative()
        
        observation = self.get_obs()
        # self.__node.get_logger().info(f"Observation vec: {observation['target_vec']}")
        if self.is_reached_goal():
            self.success_counter += 1
            reward = 20
            terminated = True
            # self.__node.get_logger().info(f"Reached goal!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        elif self.colision_flag:
            reward = -10
            terminated = True
        elif self.distance_to_goal() > 2*max(self.boundary_shape):
            reward = -5
            terminated = True
            # self.__node.get_logger().info(f"Terminated, too far from goal")

        elif self.warning_flag:
            reward = -2
            terminated = False
        else:
            reward = -1
            terminated = False
    
        # self.__node.get_logger().info(f"Flags: {self.warning_flag}  {self.colision_flag}")

        truncated = False
        info = {}
        self.reset_flag()
        # self.env_visualizer.update(self.gps_position[0:2], self.goal_point)
        # self.env_visualizer.show()
        return observation, reward, terminated, truncated, info

    def reset_flag(self, colision=False):
        self.warning_flag = False
        if colision:
            self.colision_flag = False

    def reset(self, seed=None, options=None):
        # self.__node.get_logger().info("Gym enviroment reset")
        self.ep_counter += 1
        self.env_randomizer.randomize_enviroment(change_propability=0.9)
        self.goal_point = self.env_randomizer.randomize_goal_point(change_propability=0.9)
        # print(f"goal: {self.goal_point}")
        self.set_drone_pose(self.drone_init_translation, self.drone_init_rotation, reset_physics=True)
        self.reset_flag(colision=True)

        info = {}
        observation = self.get_obs()
        return observation, info

    def __del__(self):
        self.__node.destroy_node()
        rclpy.shutdown()
        
# def main(args=None):

#     drone = DepthCameraDroneNavigation_v0()

#     # rclpy.spin_until_future_complete(action_client, future)
#     while True:
#         drone.step(0)    

#     # rclpy.shutdown()


# if __name__ == '__main__':
#     main()