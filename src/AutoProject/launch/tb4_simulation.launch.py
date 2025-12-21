import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 1. Setup Paths
    # Ensure 'AutoProject' is the name of your actual ROS 2 package
    auto_pkg_path = get_package_share_directory('AutoProject')
    tb4_desc_path = get_package_share_directory('turtlebot4_description')

    # Path to your NEW sdf file (the one with 'doubleLane' name inside)
    world_file = os.path.join(auto_pkg_path, 'worlds', 'doubleLane.sdf')

    # Xacro file for TB4
    xacro_file = os.path.join(tb4_desc_path, 'urdf', 'standard', 'turtlebot4.urdf.xacro')

    # 2. Process Robot Description
    robot_description_content = Command(['xacro ', xacro_file, ' gazebo:=ignition'])

    return LaunchDescription([
        # Start Gazebo - Added '-r' flag so it DOES NOT start automatically
        ExecuteProcess(
            cmd=['gz', 'sim', world_file], 
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                # Use the 'name' attribute from your SDF <world name="doubleLane">
                '-world', 'doubleLane', 
                '-topic', 'robot_description',
                '-name', 'turtlebot4',
                # Right lane start: x=-4.5 (beginning), y=-0.45 (right lane center), z=0
                '-x', '-4.8', '-y', '-0.225', '-z', '0.0'
            ],
            output='screen'
        ),

        # Bridge for Velocity and Clock
        # This connects ROS 2 /cmd_vel to Gazebo Sim /cmd_vel
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
		'/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ],
            output='screen'
        )
    ])
