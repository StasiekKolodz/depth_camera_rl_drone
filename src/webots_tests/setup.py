from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'webots_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name , glob('webots_vehicle.py')),
        ('share/' + package_name + '/resource', ['resource/robot_bbox.urdf']),
        ('share/' + package_name + '/resource', ['resource/ghost_robot.urdf']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stas',
    maintainer_email='stas.kolodziejczyk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_bbox_driver=webots_tests.robot_bbox_driver",
            "ghost_robot_driver=webots_tests.ghost_robot_driver",
            "supervisor_driver=webots_tests.supervisor_driver",
        ],
    },
)
