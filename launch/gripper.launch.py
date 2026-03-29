from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('gripper_ros2')

    return LaunchDescription([
        DeclareLaunchArgument('port',    default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate',default_value='921600'),

        Node(
            package='gripper_ros2',
            executable='gripper_ros_bridge',
            name='gripper_ros_bridge',
            namespace='gripper',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'default_params.yaml'),
                {
                    'port':    LaunchConfiguration('port'),
                    'baudrate':LaunchConfiguration('baudrate'),
                }
            ],
        )
    ])