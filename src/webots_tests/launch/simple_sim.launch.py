import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_tests')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot_bbox.urdf')
    ghost_description_path = os.path.join(package_dir, 'resource', 'ghost_robot.urdf')
    supervisor_description_path = os.path.join(package_dir, 'resource', 'supervisor.urdf')
    world_path = os.path.join("/home/stas/Dron/tools/ardupilot/libraries/SITL/examples/Webots_Python/worlds/", 'iris_depth_camera_touch_sensor.wbt')

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True,
    )
    my_robot_driver = WebotsController(
        robot_name='Iris',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True,
    )
    ghost_robot_driver = WebotsController(
        robot_name='ghost_robot',
        parameters=[
            {'robot_description': ghost_description_path},
        ],
        respawn=True,
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        ghost_robot_driver,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])