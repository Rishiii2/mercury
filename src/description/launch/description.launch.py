from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    
    declare_xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        description='Path to the xacro file'
    )

    pkg_prefix = get_package_prefix('description')
    process_urdf_path = os.path.join(pkg_prefix, 'lib', 'description', 'process_urdf')

    return LaunchDescription([
        declare_xacro_file_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['python3', ' ', process_urdf_path, ' ', LaunchConfiguration('xacro_file')]),
                    value_type=str
                )
            },
            {'use_sim_time': True}],
            output='screen'
        )
    ])