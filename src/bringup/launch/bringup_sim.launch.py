from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    pkg_desc = get_package_share_directory('description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')

    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (server only)'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('simulation'),
                'launch',
                'simulation.launch.py'
            ])
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    base = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(              
                    PathJoinSubstitution([
                        FindPackageShare('bringup'),
                        'launch',
                        'bringup_base.launch.py'
                    ])
                ),
                launch_arguments={
                    'xacro_file': xacro_file
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        declare_headless_arg,
        simulation,
        base
    ])