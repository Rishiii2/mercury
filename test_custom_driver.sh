#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/rishi/Desktop/mercury/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix simulation)/share/simulation/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH

echo "Starting bringup_sim in headless mode with custom driver..."
ros2 launch bringup bringup_sim.launch.py headless:=true &
LAUNCH_PID=$!

echo "Waiting 25 seconds for nodes to start..."
sleep 25

echo "=== ROS Node List ==="
ros2 node list

echo "=== ROS Topic List ==="
ros2 topic list

echo "=== ROS Control list_controllers ==="
ros2 control list_controllers

echo "=== Topic info for /wheel_velocity_controller/commands ==="
ros2 topic info /wheel_velocity_controller/commands

echo "=== Topic info for /diff_drive_controller/odom ==="
ros2 topic info /diff_drive_controller/odom

# Kill the launch process and all its children
echo "Killing launch..."
pkill -9 -f ros2
pkill -9 -f gz
pkill -9 -f ruby
pkill -9 -f rviz
killall -9 gz-sim-system-plugin-system 2>/dev/null
