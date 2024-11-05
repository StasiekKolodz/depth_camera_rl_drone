from setuptools import find_packages, setup

package_name = 'depth_camera_mavic'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))

data_files.append(('share/' + package_name + '/worlds', [
    '../worlds/depth_mavic_world.wbt',
    '../worlds/depth_mavic_world_no_gravity.wbt',
]))

data_files.append(('share/' + package_name + '/resource', [
    'resource/depth_camera_mavic.urdf'
]))

data_files.append(('share/' + package_name + '/resource', [
    'resource/webots_sim_manager.urdf'
]))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stas',
    maintainer_email='stas.kolodziejczyk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_pose_client=depth_camera_mavic.robot_pose_client:main",
            "depth_image_subscriber=depth_camera_mavic.depth_image_subscriber:main",
        ],
    },
)
