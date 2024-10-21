
"""ROS2 Mavic 2 Pro driver. Based on cyberbotics webots_ros2 pkg. 
Extends it with depth camera and touch sensors support"""

import math
import rclpy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

from drone_interfaces.srv import GetGpsPos
from drone_interfaces.action import MoveRelative


K_VERTICAL_THRUST = 68.5    # with this thrust, the drone lifts.
K_VERTICAL_P = 3.0          # P constant of the vertical PID.
K_ROLL_P = 50.0             # P constant of the roll PID.
K_PITCH_P = 30.0            # P constant of the pitch PID.
K_YAW_P = 2.0
K_X_VELOCITY_P = 1
K_Y_VELOCITY_P = 1
K_X_VELOCITY_I = 0.01
K_Y_VELOCITY_I = 0.01
LIFT_HEIGHT = 2


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class DepthMavicDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu = self.__robot.getDevice('inertial unit')
        self.__touch_sensor_warning = self.__robot.getDevice('touch_sensor_warning')
        self.__touch_sensor_colision = self.__robot.getDevice('touch_sensor_colision')
        self.__touch_sensor_warning.enable(self.__timestep)
        self.__touch_sensor_colision.enable(self.__timestep)
        # Propellers
        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # State
        self.__target_twist = Twist()
        self.__vertical_ref = LIFT_HEIGHT
        self.__linear_x_integral = 0
        self.__linear_y_integral = 0

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('mavic_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_service(GetGpsPos, "gps_position", self.__gps_position_callback)
        self.touch_sensor_warning_publisher = self.__node.create_publisher(Bool, 'Depth_Mavic_2_PRO/touch_sensor_warning', 10)
        self.touch_sensor_colision_publisher = self.__node.create_publisher(Bool, 'Depth_Mavic_2_PRO/touch_sensor_colision', 10)

    def publish_touch_sensors(self):
        ts_warning = self.__touch_sensor_warning.getValue()
        ts_colision = self.__touch_sensor_colision.getValue()

        ts_warning_msg = Bool()
        ts_warning_msg.data = bool(ts_warning)
        ts_colision_msg = Bool()
        ts_colision_msg.data = bool(ts_colision)

        self.touch_sensor_warning_publisher.publish(ts_warning_msg)
        self.touch_sensor_colision_publisher.publish(ts_colision_msg)

    def __gps_position_callback(self, request, response):
        x, y, z = self.__gps.getValues()
        # response = Vector3.Response()
        response.x = x
        response.y = y
        response.z = z

        return response

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        roll_ref = 0
        pitch_ref = 0

        # Read sensors
        roll, pitch, _ = self.__imu.getRollPitchYaw()
        _, _, vertical = self.__gps.getValues()
        roll_velocity, pitch_velocity, twist_yaw = self.__gyro.getValues()
        velocity = self.__gps.getSpeed()
        if math.isnan(velocity):
            return
        
        # Publish touch sensors values
        self.publish_touch_sensors()

        # Allow high level control once the drone is lifted
        if vertical > 0.2:
            # Calculate velocity
            velocity_x = (pitch / (abs(roll) + abs(pitch))) * velocity
            velocity_y = - (roll / (abs(roll) + abs(pitch))) * velocity

            # High level controller (linear velocity)
            linear_y_error = self.__target_twist.linear.y - velocity_y
            linear_x_error = self.__target_twist.linear.x - velocity_x
            self.__linear_x_integral += linear_x_error
            self.__linear_y_integral += linear_y_error
            roll_ref = K_Y_VELOCITY_P * linear_y_error + K_Y_VELOCITY_I * self.__linear_y_integral
            pitch_ref = - K_X_VELOCITY_P * linear_x_error - K_X_VELOCITY_I * self.__linear_x_integral
            self.__vertical_ref = clamp(
                self.__vertical_ref + self.__target_twist.linear.z * (self.__timestep / 1000),
                max(vertical - 0.5, LIFT_HEIGHT),
                vertical + 0.5
            )
        vertical_input = K_VERTICAL_P * (self.__vertical_ref - vertical)

        # Low level controller (roll, pitch, yaw)
        yaw_ref = self.__target_twist.angular.z

        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_velocity + roll_ref
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_velocity + pitch_ref
        yaw_input = K_YAW_P * (yaw_ref - twist_yaw)

        m1 = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        m2 = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        m3 = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input
        m4 = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input

        # Apply control
        self.__propellers[0].setVelocity(-m1)
        self.__propellers[1].setVelocity(m2)
        self.__propellers[2].setVelocity(m3)
        self.__propellers[3].setVelocity(-m4)
