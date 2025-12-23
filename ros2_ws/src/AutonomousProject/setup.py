from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'AutonomousProject'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/AutonomousProject']),
        ('share/' + package_name, ['package.xml']),
        # This ensures your worlds are installed in the share directory
        ('share/' + package_name + '/worlds', [
            'worlds/doubleLane.sdf', 
            'worlds/lane_visual.sdf'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mario',
    maintainer_email='mario@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_perception = AutonomousProject.lane_perception_node:main',
            'lidar_obstacle = AutonomousProject.lidar_obstacle_node:main',
            'pid_controller = AutonomousProject.pid_controller_node:main',
            
        ],
    },
)
