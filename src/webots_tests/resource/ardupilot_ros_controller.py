#!/usr/bin/env python3

import time
import argparse
from webots_vehicle import WebotsArduVehicle
import rclpy
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import Bool

from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--motors", "-m",
                        type=str,
                        default="m1_motor, m2_motor, m3_motor, m4_motor",
                        help="Comma spaced list of motor names in ardupilot numerical order (ex --motors \"m1,m2,m3, m4\")")
    parser.add_argument("--reversed-motors", "-r",
                        type=str,
                        default=None,
                        help="Comma spaced list of motors to reverse (starting from 1, in ardupilot order)")
    parser.add_argument("--bidirectional-motors",
                        type=bool,
                        default=False,
                        help="If the motors are bidirectional (as is the case for Rovers usually)")
    parser.add_argument("--uses-propellers",
                        type=bool,
                        default=True,
                        help="Whether the vehicle uses propellers. This is important as we need to linearize thrust if so")
    parser.add_argument("--motor-cap",
                        type=float,
                        default=float('inf'),
                        help="Motor velocity cap. This is useful for the crazyflie which default has way too much power")

    parser.add_argument("--accel",
                        type=str,
                        default="accelerometer",
                        help="Webots accelerometer name")
    parser.add_argument("--imu",
                        type=str,
                        default="inertial unit",
                        help="Webots IMU name")
    parser.add_argument("--gyro",
                        type=str,
                        default="gyro",
                        help="Webots gyro name")
    parser.add_argument("--gps",
                        type=str,
                        default="gps",
                        help="Webots GPS name")

    parser.add_argument("--camera",
                        type=str,
                        default=None,
                        help="Webots Camera name (optional)")
    parser.add_argument("--camera-fps",
                        type=int,
                        default=10,
                        help="Camera FPS. Note lower FPS is faster")
    parser.add_argument("--camera-topic",
                        type=str,
                        default=None,
                        help="Ros topic to stream grayscale camera images to. "
                             "If no topic is supplied the camera will not be streamed.")

    parser.add_argument("--rangefinder",
                        type=str,
                        default=None,
                        help="Webots RangeFinder name (optional)")
    parser.add_argument("--rangefinder-fps",
                        type=int,
                        default=10,
                        help="rangefinder FPS. Note lower FPS is faster")
    parser.add_argument("--rangefinder-topic",
                        type=str,
                        default=None,
                        help="Ros topic to stream grayscale rangefinder images to. "
                             "If no topic is supplied the rangefinder will not be streamed.")

    parser.add_argument("--instance", "-i",
                        type=int,
                        default=0,
                        help="Drone instance to match the SITL. This allows multiple vehicles")
    parser.add_argument("--sitl-address",
                        type=str,
                        default="127.0.0.1",
                        help="IP address of the SITL (useful with WSL2 eg \"172.24.220.98\")")

    return parser.parse_args()

    # ...
class WebotsRosDriver(node):
    def __init__(self, webots_ardu_vechicle):
        super().__init__('webots_ros_driver')

        self.webots_ardu_vechicle = webots_ardu_vechicle
        self.touch_sensor = webots_ardu_vechicle.robot.getDevice('touch sensor')
        self.touch_sensor .enable(webots_ardu_vechicle._timestep)

        self.camera_publisher = self.create_publisher(Image, 'camera')
        self.touch_sensor_publisher = self.create_publisher(Bool, 'touch_sensor_1')

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        def publish_touch_sensor(self):
            touch_msg = Bool()
            touch_msg.data = self.touch_sensor.get_value()
            self.touch_sensor_publisher.publish(touch_msg)
            self.get_logger().info("Touch sensor published")

        def timer_callback(self):
            self.publish_touch_sensor()


if __name__ == "__main__":
    args = get_args()

    # parse string arguments into lists
    motors = [x.strip() for x in args.motors.split(',')]
    if args.reversed_motors:
        reversed_motors = [int(x) for x in args.reversed_motors.split(",")]
    else:
        reversed_motors = []

    vehicle = WebotsArduVehicle(motor_names=motors,
                                reversed_motors=reversed_motors,
                                accel_name=args.accel,
                                imu_name=args.imu,
                                gyro_name=args.gyro,
                                gps_name=args.gps,
                                camera_name=args.camera,
                                camera_fps=args.camera_fps,
                                #Camera handled by ROS
                                camera_stream_port=None,
                                rangefinder_name=args.rangefinder,
                                rangefinder_fps=args.rangefinder_fps,
                                #RangeFinder handled by ROS
                                rangefinder_stream_port=None,
                                instance=args.instance,
                                motor_velocity_cap=args.motor_cap,
                                bidirectional_motors=args.bidirectional_motors,
                                uses_propellers=args.uses_propellers,
                                sitl_address=args.sitl_address)

    # User code (ex: connect via drone kit and take off)

    while vehicle.webots_connected():
        rclpy.init(args=args)
        ros_driver  = WebotsRosDriver()
        rclpy.spin(ros_driver)
        ros_driver.destroy_node()
        rclpy.shutdown()

        time.sleep(1)
        
