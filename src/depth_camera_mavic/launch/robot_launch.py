#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Mavic 2 Pro driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('depth_camera_mavic')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'depth_camera_mavic.urdf')
    mavic_driver = WebotsController(
        robot_name='Depth_Mavic_2_PRO',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True
    )

    sim_manager_description_path = os.path.join(package_dir, 'resource', 'webots_sim_manager.urdf')
    ros2_webots_sim_manager = WebotsController(
        robot_name='ros2_sim_manager',
        parameters=[
            {'robot_description': sim_manager_description_path},
        ],
        respawn=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='depth_mavic_world_no_gravity.wbt',
            description='Choose one of the world files from `/webots_ros2_mavic/worlds` directory'
        ),
        webots,
        webots._supervisor,
        mavic_driver,
        ros2_webots_sim_manager,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
