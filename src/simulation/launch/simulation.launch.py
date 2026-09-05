from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    import subprocess
    subprocess.run(["pkill", "-9", "-f", "ign gazebo"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stderr=subprocess.DEVNULL)

    pkg_desc = get_package_share_directory('description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    
    pkg_sim = get_package_share_directory('simulation')

    world_file = os.path.join(
        pkg_sim,
        'worlds',
        'mercury.sdf'
    )
    images_dir = os.path.join(pkg_sim, 'models', 'images')

    import re
    with open(world_file, 'r') as f:
        world_content = f.read()

    for i in range(1, 7):
        world_content = world_content.replace(
            f'photo{i}.jpg',
            f'file://{images_dir}/photo{i}.jpg'
        )

    import shutil
    is_ign = bool(shutil.which('ign'))
    if is_ign:
        gz_cmd = ['ign', 'gazebo', '-r', world_file]
        # Replace Gazebo Harmonic / Jazzy system plugin names with Ignition Fortress / Humble plugin names
        world_content = world_content.replace('gz-sim-physics-system', 'ignition-gazebo-physics-system')
        world_content = world_content.replace('gz::sim::systems::Physics', 'ignition::gazebo::systems::Physics')
        world_content = world_content.replace('gz-sim-user-commands-system', 'ignition-gazebo-user-commands-system')
        world_content = world_content.replace('gz::sim::systems::UserCommands', 'ignition::gazebo::systems::UserCommands')
        world_content = world_content.replace('gz-sim-scene-broadcaster-system', 'ignition-gazebo-scene-broadcaster-system')
        world_content = world_content.replace('gz::sim::systems::SceneBroadcaster', 'ignition::gazebo::systems::SceneBroadcaster')
        world_content = world_content.replace('gz-sim-contact-system', 'ignition-gazebo-contact-system')
        world_content = world_content.replace('gz::sim::systems::Contact', 'ignition::gazebo::systems::Contact')
        world_content = world_content.replace('gz-sim-navsat-system', 'ignition-gazebo-navsat-system')
        world_content = world_content.replace('gz::sim::systems::NavSat', 'ignition::gazebo::systems::NavSat')
        world_content = world_content.replace('gz-sim-imu-system', 'ignition-gazebo-imu-system')
        world_content = world_content.replace('gz::sim::systems::Imu', 'ignition::gazebo::systems::Imu')
        world_content = world_content.replace('gz-sim-sensors-system', 'ignition-gazebo-sensors-system')
        world_content = world_content.replace('gz::sim::systems::Sensors', 'ignition::gazebo::systems::Sensors')
    else:
        gz_cmd = ['gz', 'sim', '-r', world_file]

    import tempfile
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
    tmp.write(world_content)
    tmp.flush()
    world_file = tmp.name

    if is_ign:
        gz_cmd[-1] = world_file

    models_path = os.path.join(pkg_sim, 'models') + ':' + os.path.join(pkg_sim, 'models', 'images')

    gz_process = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2'
    )

    return LaunchDescription([

        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),

        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '/opt/ros/humble/lib'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/humble/lib'),

        gz_process,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_process,
                on_exit=[Shutdown()]
            )
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'mercury',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
                '-Y', '0.0',
                '-allow_renaming', 'false',
            ],
            parameters=[{'use_sim_time': True}],    
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['turret_controller'],
                    output='screen'
                ),
            ]
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/turret_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='bringup',
            executable='twist_to_stamped',
            name='twist_to_stamped',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])