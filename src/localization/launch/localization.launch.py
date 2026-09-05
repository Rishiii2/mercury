from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization'),
                'launch',
                'ekf.launch.py'
            ])
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization'),
                'launch',
                'slam.launch.py'
            ])
        )
    )

    # Note: async_slam_toolbox_node is a standard rclcpp node (not a lifecycle node),
    # so it starts automatically without nav2_lifecycle_manager.

    # --- GPS localization pipeline ---
    pkg_loc = get_package_share_directory('localization')
    navsat_params = os.path.join(pkg_loc, 'config', 'navsat_transform.yaml')

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_params],
        remappings=[
            ('gps/fix', '/gps_fixed'),
            ('imu', '/imu'),
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    gps_relay = Node(
        package='simulation',
        executable='gps_relay',
        name='gps_relay',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        ekf,
        slam,
        navsat_transform,
        gps_relay,
    ])