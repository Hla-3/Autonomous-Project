import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 1. Setup Paths
    auto_pkg_path = get_package_share_directory('AutoProject')
    # CHANGED: Use turtlebot3_description instead of turtlebot4
    tb3_desc_path = get_package_share_directory('turtlebot3_description')

    world_file = os.path.join(auto_pkg_path, 'worlds', 'lane_visual.sdf')

    # CHANGED: Path to TB3 Burger URDF
    xacro_file = os.path.join(tb3_desc_path, 'urdf', 'turtlebot3_burger.urdf')

    # 2. Process Robot Description
    # Note: TB3 URDFs are usually plain URDF or slightly different xacro; 
    # reading it directly is safest for standard TB3 packages.
    with open(xacro_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Start Gazebo with the lane world (-r starts it running automatically)
        ExecuteProcess(cmd=['gz', 'sim', world_file], output='screen'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': True
            }]
        ),

        # Spawn Robot (Burger)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'lane_world',
                '-topic', 'robot_description',
                '-name', 'turtlebot3_burger', # CHANGED name
                '-x', '-4.8', '-y', '0', '-z', '0.01' # Updated coordinates
            ],
            output='screen'
        ),

        # Bridge for Velocity, Clock, and LIDAR
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Cmd Vel bridge
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                # Clock bridge
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # LIDAR bridge - This is what allows you to read /scan
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                # TF bridge for Lidar transforms
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'
        )
    ])