#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/rishi/Desktop/mercury/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix simulation)/share/simulation/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib
export PYTHONNOUSERSITE=1

echo "Starting simulation in background..."
ros2 launch bringup bringup_sim.launch.py &
LAUNCH_PID=$!

echo "Waiting 30 seconds for nodes and controllers to spin up..."
sleep 30

echo "=== Active Controllers ==="
ros2 control list_controllers

echo "=== Initial Odometry (Custom Driver) ==="
ros2 topic echo /diff_drive_controller/odom --once

echo "=== Initial Ground Truth Odom (Gazebo) ==="
ros2 topic echo /odom --once

echo "=== Publishing cmd_vel_nav to move forward ==="
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "Waiting 5 seconds..."
sleep 5

echo "=== Odometry after moving (Custom Driver) ==="
ros2 topic echo /diff_drive_controller/odom --once

echo "=== Ground Truth Odom after moving (Gazebo) ==="
ros2 topic echo /odom --once

echo "=== Stopping simulation ==="
kill -INT $LAUNCH_PID
sleep 3
kill -9 $LAUNCH_PID 2>/dev/null
echo "Test done."
