import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # 1. Setup Paths
    auto_pkg_path = get_package_share_directory('AutoProject')
    # Using the standard turtlebot3_description package
    tb3_burger_desc_path = get_package_share_directory('turtlebot3_description')

    world_file = os.path.join(auto_pkg_path, 'worlds', 'doubleLane.sdf')
    xacro_file = os.path.join(tb3_burger_desc_path, 'urdf', 'turtlebot3_burger.urdf')

    # 2. Robot Description
    with open(xacro_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Start Gazebo Sim with the doubleLane world
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }]
        ),

        # Spawn Robot at specific coordinates
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'turtlebot3_burger',
                '-topic', 'robot_description',
                '-x', '-4.8',
                '-y', '-0.225',
                '-z', '0.01' # Slightly above 0 to avoid floor collision on spawn
            ],
            output='screen'
        ),

        # Bridge for Cmd_Vel, Clock, and LIDAR
        # Note: TB3 Lidar usually publishes to /scan
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Velocity bridge
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # Clock bridge
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Lidar bridge (This allows you to "read" the lidar)
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                # TF bridge (Optional but recommended for Rviz)
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'
        )
    ])