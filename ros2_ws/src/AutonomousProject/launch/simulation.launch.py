import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'AutonomousProject'
    
    # Path to your SDF world file
    world_file_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'lane_visual.sdf'
    )

    # 1. Launch Gazebo Sim with the world file
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file_path],
        output='screen'
    )

    # 2. ROS-GZ Bridge (Camera and Cmd_vel)
    # Note: We combine them into one bridge node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/lane_detected@std_msgs/msg/Bool@gz.msgs.Boolean' # Optional if you want to see it in Gazebo
        ],
        output='screen'
    )

    # 3. Lane Perception Node
    lane_perception = Node(
        package=package_name,
        executable='lane_perception', # Matches entry_point in setup.py
        output='screen'
    )

    # 4. PID Controller Node
    pid_controller = Node(
        package=package_name,
        executable='pid_controller', # Matches entry_point in setup.py
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        lane_perception,
        pid_controller
    ])
