#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/rishi/Desktop/mercury/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix simulation)/share/simulation/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH

echo "Starting simulation in background..."
ros2 launch bringup bringup_sim.launch.py headless:=true &
LAUNCH_PID=$!

echo "Waiting 30 seconds for nodes and controllers to spin up..."
sleep 30

echo "=== Active Controllers ==="
ros2 control list_controllers

echo "=== Initial Odometry (Custom Driver) ==="
timeout 15 ros2 topic echo /diff_drive_controller/odom --once || echo "Timeout or no message on /diff_drive_controller/odom"

echo "=== Initial Ground Truth Odom (Gazebo) ==="
timeout 15 ros2 topic echo /odom --once || echo "Timeout or no message on /odom"

echo "=== Publishing cmd_vel_nav to move forward ==="
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "Waiting 5 seconds..."
sleep 5

echo "=== Odometry after moving (Custom Driver) ==="
timeout 15 ros2 topic echo /diff_drive_controller/odom --once || echo "Timeout or no message on /diff_drive_controller/odom"

echo "=== Ground Truth Odom after moving (Gazebo) ==="
timeout 15 ros2 topic echo /odom --once || echo "Timeout or no message on /odom"

echo "=== Stopping simulation ==="
pkill -9 -f ros2
pkill -9 -f gz
pkill -9 -f ruby
pkill -9 -f rviz
killall -9 gz-sim-system-plugin-system 2>/dev/null
echo "Test done."
