import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 1. Setup Paths
    auto_pkg_path = get_package_share_directory('AutoProject')
    tb4_desc_path = get_package_share_directory('turtlebot4_description')

    world_file = os.path.join(auto_pkg_path, 'worlds', 'lane_visual.sdf')

    # EXACT PATH from your 'find' command
    xacro_file = os.path.join(tb4_desc_path, 'urdf', 'standard', 'turtlebot4.urdf.xacro')

    # 2. Process Robot Description (Xacro to URDF)
    # We add gazebo:=ignition to ensure it loads the correct plugins for the simulator
    robot_description_content = Command(['xacro ', xacro_file, ' gazebo:=ignition'])

    return LaunchDescription([
        # Start Gazebo with the lane world
        ExecuteProcess(cmd=['gz', 'sim', world_file], output='screen'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
	# Spawn the Differential Drive Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diffdrive_controller'],
            output='screen',
        ),

        # Spawn the Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'lane_world',
                '-topic', 'robot_description',
                '-name', 'turtlebot4',
                '-x', '-5', '-y', '0', '-z', '0'
            ],
            output='screen'
        ),

        # Bridge for Velocity (Allows you to drive)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/turtlebot4/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
		       '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        )
    ])
